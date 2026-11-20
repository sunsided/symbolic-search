//! `SymBA*`: bidirectional symbolic A* with perimeter abstraction
//! heuristics (Torralba et al. 2014, 2016).
//!
//! ```mermaid
//! graph TD
//!     Pick[pick the direction with the smaller next bucket] --> Big{bucket over the frontier limit and abstract work left?}
//!     Big -- no --> Expand[expand concretely, check meetings]
//!     Big -- yes --> Resume[resume or start this side's abstract search]
//!     Resume --> Slice[expand up to one slice the other way]
//!     Slice -- paused --> Install[h = max of frozen perimeters and the current one, refile open]
//!     Slice -- over the abstract limit --> Freeze[freeze it, next level drops one more variable]
//!     Slice -- exhausted --> Done[freeze it, no more abstract work on this side]
//!     Freeze --> Install
//!     Done --> Install
//!     Install --> Expand
//!     Expand --> Pick
//! ```
//!
//! Both concrete frontiers start blind, so the search is bidirectional
//! Dijkstra until a frontier grows past the frontier node limit. From then
//! on, each time that side is about to expand, it first gets one slice of
//! abstract work: an abstract search in the opposite direction (backward from
//! the abstract goal for the forward frontier, forward from the abstract
//! initial state for the backward frontier). Each side keeps its abstract
//! search between slices and resumes it instead of starting over.
//!
//! Level `k` of a side's hierarchy drops `k` variables, non-goal variables
//! first, so each level is coarser and cheaper than the last. An abstract
//! search whose next bucket passes the abstract node limit is frozen and the
//! next level starts. An exhausted search is exact for its abstraction, and
//! coarser levels can only give smaller values, so the side stops abstract
//! work.
//!
//! The side's heuristic is the maximum of the frozen perimeters and the
//! current search's perimeter. A perimeter at any pause point is admissible
//! and consistent (closed states get their abstract distance, all others the
//! smallest open cost), and so is the maximum, which keeps the bidirectional
//! termination bound valid. Blind heuristics are not installed.
//!
//! The expansion and time budget counts concrete and abstract expansions and
//! is checked before each of them.
//!
//! Simplifications against the full planner: abstractions are fixed variable
//! drops instead of merge-and-shrink or chosen patterns, and the step choice
//! and slices use node counts and expansion counts instead of time
//! estimates.

use std::cell::OnceCell;
use std::time::Instant;

use sas_plus::VarId;

use crate::backend::Backend;
use crate::model::abstraction::perimeter_of;
use crate::model::frontier::{Frontier, Strategy};
use crate::model::heuristic::Partition;
use crate::model::space::Space;
use crate::model::{Direction, Model};
use crate::{Error, Solution, SymbaConfig};

/// Counters of one `SymBA*` run, for tests and tuning.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub(crate) struct Stats {
    /// Informative (not blind) heuristics installed into concrete frontiers.
    pub installed: usize,
    /// Slices that resumed a paused abstract search.
    pub resumed: usize,
    /// Abstract searches frozen for passing the abstract node limit.
    pub frozen: usize,
}

/// The expansion and time budget of a run.
struct Budget {
    /// Expansions left, `None` for no limit.
    expansions: Option<u64>,
    /// When the run must stop, `None` for no limit.
    deadline: Option<Instant>,
}

impl Budget {
    /// The budget of `config`, with the clock starting now.
    fn new(config: &SymbaConfig) -> Self {
        Self {
            expansions: config.max_expansions,
            deadline: config
                .time_limit
                .and_then(|t| Instant::now().checked_add(t)),
        }
    }

    /// Pays for one expansion.
    ///
    /// # Errors
    ///
    /// [`Error::BudgetExhausted`] when no expansion or time is left.
    fn charge(&mut self) -> Result<(), Error> {
        // The bidirectional driver fills in the best plan and the bound.
        let exhausted = Error::BudgetExhausted {
            best: None,
            lower_bound: 0,
        };
        let out_of_time = self.deadline.is_some_and(|d| Instant::now() >= d);
        match &mut self.expansions {
            _ if out_of_time => Err(exhausted),
            Some(0) => Err(exhausted),
            Some(left) => {
                *left -= 1;
                Ok(())
            }
            None => Ok(()),
        }
    }
}

/// How one slice of abstract work ended.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Outcome {
    /// The slice was used up; the search resumes next time.
    Paused,
    /// The next bucket passed the abstract node limit; the search is frozen.
    TooHard,
    /// Open ran empty; the search is exact for its abstraction.
    Exhausted,
}

/// The abstraction hierarchy of one concrete side and its live abstract
/// search.
struct Hierarchy<'s, B: Backend> {
    /// Direction of the abstract searches: opposite to the concrete side.
    direction: Direction,
    /// Level the next fresh abstract search runs on.
    next_level: usize,
    /// The paused abstract search, if any.
    current: Option<Frontier<'s, B>>,
    /// Maximum of the perimeters of frozen searches; `None` before the
    /// first one.
    frozen: Option<Partition<B>>,
    /// Whether this side does no more abstract work.
    done: bool,
}

/// What a [`Hierarchy`] needs from the run: the model, the lazily built
/// projected spaces, the drop order and the parameters.
struct Levels<'s, B: Backend> {
    /// The model whose concrete space is abstracted.
    model: &'s Model<B>,
    /// The variables in drop order; level `k` drops the first `k`.
    drop_order: &'s [VarId],
    /// Projected space per level, built on first use. Index 0 is unused.
    // shortcut: visited levels stay alive for the whole run; drop frozen
    // ones once memory matters.
    spaces: &'s [OnceCell<Space<B>>],
    /// The run's parameters.
    config: &'s SymbaConfig,
}

impl<B: Backend> Model<B> {
    /// `SymBA*` with the parameters of `config`.
    ///
    /// Returns an optimal plan in execution order, or `None` when the goal is
    /// unreachable, plus the run's counters.
    ///
    /// # Errors
    ///
    /// - [`Error::BudgetExhausted`] when the budget of `config` runs out.
    /// - [`Error::OutOfMemory`] when the node table fills up.
    pub(crate) fn symba(&self, config: &SymbaConfig) -> Result<(Option<Solution>, Stats), Error> {
        let b = &self.backend;
        let drop_order = self.drop_order();
        let spaces: Vec<OnceCell<Space<B>>> = drop_order.iter().map(|_| OnceCell::new()).collect();
        let levels = Levels {
            model: self,
            drop_order: &drop_order,
            spaces: &spaces,
            config,
        };
        let mut sides = [Direction::Backward, Direction::Forward].map(Hierarchy::new);
        let mut budget = Budget::new(config);
        let mut stats = Stats::default();

        let mut hook = |frontier: &mut Frontier<'_, B>| -> Result<(), Error> {
            budget.charge()?;
            if frontier.next_size() <= config.frontier_node_limit {
                return Ok(());
            }

            let side = &mut sides[usize::from(frontier.direction() == Direction::Backward)];
            side.work(&levels, frontier, &mut budget, &mut stats)
        };

        let side = |direction| {
            Frontier::new(
                b,
                &self.concrete,
                direction,
                Partition::zero(b),
                Strategy::Cheapest,
            )
        };
        let plan = self.bidirectional(
            side(Direction::Forward)?,
            side(Direction::Backward)?,
            &mut hook,
        )?;

        Ok((plan, stats))
    }

    /// The variables in the order abstractions drop them: non-goal variables
    /// first, each group from the last variable id down.
    fn drop_order(&self) -> Vec<VarId> {
        let is_goal = |var: &VarId| self.goal_vars.binary_search(var).is_ok();
        let all = (0..self.encoding.num_vars()).rev();

        all.clone()
            .filter(|var| !is_goal(var))
            .chain(all.filter(is_goal))
            .collect()
    }
}

impl<'s, B: Backend> Levels<'s, B> {
    /// The projected space of `level`, built on first use.
    ///
    /// # Errors
    ///
    /// Out of memory in the backend.
    fn space(&self, level: usize) -> Result<&'s Space<B>, Error> {
        let cell: &'s OnceCell<Space<B>> = &self.spaces[level];
        if cell.get().is_none() {
            let space = self.model.project(&self.drop_order[..level])?;
            let _ = cell.set(space);
        }

        Ok(cell.get().expect("the space was just built"))
    }
}

impl<'s, B: Backend> Hierarchy<'s, B> {
    /// An empty hierarchy whose abstract searches run in `direction`.
    fn new(direction: Direction) -> Self {
        Self {
            direction,
            next_level: 1,
            current: None,
            frozen: None,
            done: false,
        }
    }

    /// Runs one slice of abstract work and installs the resulting heuristic
    /// into the concrete `frontier`.
    ///
    /// # Errors
    ///
    /// [`Error::BudgetExhausted`] or [`Error::OutOfMemory`].
    fn work(
        &mut self,
        levels: &Levels<'s, B>,
        frontier: &mut Frontier<'_, B>,
        budget: &mut Budget,
        stats: &mut Stats,
    ) -> Result<(), Error> {
        if self.done {
            return Ok(());
        }

        let Some(mut search) = self.resume_or_start(levels, stats)? else {
            self.done = true;
            return Ok(());
        };

        let outcome = run_slice(&mut search, levels.config, budget)?;
        let b = levels.model.backend();
        stats.frozen += usize::from(outcome == Outcome::TooHard);
        let heuristic = self.absorb(b, search, outcome)?;
        if heuristic.is_blind(b) {
            return Ok(());
        }

        frontier.set_heuristic(heuristic)?;
        stats.installed += 1;

        Ok(())
    }

    /// The paused abstract search, or a fresh one on the next level;
    /// `None` when no level is left.
    ///
    /// # Errors
    ///
    /// Out of memory in the backend.
    fn resume_or_start(
        &mut self,
        levels: &Levels<'s, B>,
        stats: &mut Stats,
    ) -> Result<Option<Frontier<'s, B>>, Error> {
        if let Some(search) = self.current.take() {
            stats.resumed += 1;
            return Ok(Some(search));
        }

        if self.next_level >= levels.drop_order.len() {
            return Ok(None);
        }

        let b = levels.model.backend();
        let space = levels.space(self.next_level)?;
        self.next_level += 1;
        let search = Frontier::new(
            b,
            space,
            self.direction,
            Partition::zero(b),
            Strategy::Cheapest,
        )?;

        Ok(Some(search))
    }

    /// Files the slice's `search` by its `outcome` (kept when paused, frozen
    /// otherwise) and returns the side's heuristic: the frozen maximum with
    /// the paused perimeter on top.
    ///
    /// # Errors
    ///
    /// Out of memory in the backend.
    fn absorb(
        &mut self,
        b: &B,
        search: Frontier<'s, B>,
        outcome: Outcome,
    ) -> Result<Partition<B>, Error> {
        let perimeter = perimeter_of(b, &search)?;
        let combined = match &self.frozen {
            Some(frozen) => frozen.max(b, &perimeter)?,
            None => perimeter,
        };

        match outcome {
            Outcome::Paused => self.current = Some(search),
            Outcome::TooHard => self.frozen = Some(combined.clone()),
            Outcome::Exhausted => {
                self.frozen = Some(combined.clone());
                self.done = true;
            }
        }

        Ok(combined)
    }
}

/// Expands `search` for at most one slice of `config` (at least one
/// expansion unless open is empty), and returns how it ended.
///
/// # Errors
///
/// [`Error::BudgetExhausted`] or [`Error::OutOfMemory`].
fn run_slice<B: Backend>(
    search: &mut Frontier<'_, B>,
    config: &SymbaConfig,
    budget: &mut Budget,
) -> Result<Outcome, Error> {
    let slice = config.abstract_slice.max(1);
    let mut expanded = 0;
    while expanded < slice {
        budget.charge()?;
        let Some((g, states)) = search.pop()? else {
            return Ok(Outcome::Exhausted);
        };

        search.expand(g, &states)?;
        expanded += 1;
        if search.next_size() > config.abstract_node_limit {
            return Ok(Outcome::TooHard);
        }
    }

    // A search that stops right at exhaustion is exact; say so.
    if search.is_exhausted() {
        return Ok(Outcome::Exhausted);
    }

    Ok(Outcome::Paused)
}

#[cfg(test)]
mod tests {
    use core::time::Duration;

    use super::*;
    use crate::Config;
    use crate::backend::bcdd::Oxidd;

    /// Tiny tables and a random task with conditional effects.
    fn model(seed: u64) -> (sas_plus::Task, Model<Oxidd>) {
        let params = sas_plus::random::RandomTaskParams {
            max_domain: 5,
            max_conditions: 2,
            ..sas_plus::random::RandomTaskParams::default()
        };
        let task = sas_plus::random::random_task(seed, params);
        let config = Config {
            node_capacity: 1 << 12,
            cache_capacity: 1 << 8,
            ..Config::default()
        };
        let model = Model::new(&task, &config).unwrap();

        (task, model)
    }

    /// An abstract search paused after every expansion and resumed until
    /// exhaustion ends with the same perimeter as one uninterrupted run, and
    /// every intermediate perimeter is pointwise below the final one.
    #[test]
    fn sliced_abstract_search_equals_uninterrupted_run() {
        let config = SymbaConfig {
            abstract_node_limit: usize::MAX,
            abstract_slice: 1,
            ..SymbaConfig::default()
        };
        for seed in 0..16 {
            let (_, model) = model(seed);
            let b = model.backend();
            let space = model.project(&model.drop_order()[..1]).unwrap();
            for direction in [Direction::Forward, Direction::Backward] {
                let whole = model.perimeter(&space, direction, usize::MAX).unwrap();
                let mut search =
                    Frontier::new(b, &space, direction, Partition::zero(b), Strategy::Cheapest)
                        .unwrap();
                let mut budget = Budget::new(&config);
                let mut pauses = Vec::new();
                while run_slice(&mut search, &config, &mut budget).unwrap() == Outcome::Paused {
                    pauses.push(perimeter_of(b, &search).unwrap());
                }

                let last = perimeter_of(b, &search).unwrap();
                assert!(last.buckets() == whole.buckets(), "seed {seed}");
                for paused in pauses {
                    assert!(paused.max(b, &last).unwrap().buckets() == last.buckets());
                }
            }
        }
    }

    /// Small slices and a small abstract limit make `SymBA*` both resume
    /// and freeze abstract searches, and the plans still cost as much as
    /// bidirectional Dijkstra's.
    #[test]
    fn small_slices_resume_and_stay_optimal() {
        let config = SymbaConfig {
            frontier_node_limit: 1,
            abstract_node_limit: 8,
            abstract_slice: 2,
            ..SymbaConfig::default()
        };
        let (mut resumed, mut frozen) = (0, 0);
        for seed in 0..16 {
            let (task, model) = model(seed);
            let (plan, stats) = model.symba(&config).unwrap();
            let (reference, _) = model
                .symba(&SymbaConfig {
                    frontier_node_limit: usize::MAX,
                    ..config
                })
                .unwrap();

            resumed += stats.resumed;
            frozen += stats.frozen;
            assert_eq!(cost(&task, plan), cost(&task, reference), "seed {seed}");
        }

        assert!(
            resumed > 0 && frozen > 0,
            "resumed {resumed}, frozen {frozen}"
        );
    }

    /// The budget stops a run at the deterministic extremes: no expansions,
    /// no time; an ample budget changes nothing.
    #[test]
    fn budget_stops_the_run() {
        let (task, model) = model(3);
        let base = SymbaConfig {
            frontier_node_limit: 1,
            abstract_slice: 1,
            ..SymbaConfig::default()
        };
        let starved = [
            SymbaConfig {
                max_expansions: Some(0),
                ..base
            },
            SymbaConfig {
                time_limit: Some(Duration::ZERO),
                ..base
            },
        ];
        for config in starved {
            let error = model.symba(&config).unwrap_err();
            assert!(matches!(error, Error::BudgetExhausted { best: None, .. }));
        }

        let ample = SymbaConfig {
            max_expansions: Some(u64::MAX),
            time_limit: Some(Duration::from_secs(3600)),
            ..base
        };
        let plan = |config| model.symba(&config).unwrap().0;
        assert_eq!(cost(&task, plan(ample)), cost(&task, plan(base)));
    }

    /// The cost of `solution`, checked against the validator.
    fn cost(task: &sas_plus::Task, solution: Option<Solution>) -> Option<u64> {
        solution.map(|s| {
            assert_eq!(task.validate_plan(&s.plan), Some(s.cost));
            s.cost
        })
    }

    /// Every budget cut-off reports a lower bound at most the optimum and,
    /// when it has one, a valid plan at least the optimum. Some cut-offs
    /// come after a first meeting, so a plan is reported.
    #[test]
    fn exhausted_budget_reports_best_plan_and_bound() {
        let mut with_plan = 0;
        for seed in 0..64 {
            let (task, model) = model(seed);
            let base = SymbaConfig {
                frontier_node_limit: usize::MAX,
                ..SymbaConfig::default()
            };
            let Some(optimum) = cost(&task, model.symba(&base).unwrap().0) else {
                continue;
            };

            for budget in 0..40 {
                let config = SymbaConfig {
                    max_expansions: Some(budget),
                    ..base
                };
                let Err(Error::BudgetExhausted { best, lower_bound }) = model.symba(&config) else {
                    continue;
                };

                assert!(lower_bound <= optimum, "seed {seed}, budget {budget}");
                if let Some(best) = cost(&task, best) {
                    assert!(best >= optimum);
                    with_plan += 1;
                }
            }
        }

        assert!(with_plan > 0);
    }
}
