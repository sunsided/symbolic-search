//! Bidirectional cost-optimal search: a forward and a backward frontier that
//! meet in the middle.
//!
//! Each iteration expands the direction whose next bucket has fewer BDD
//! nodes. Two checks find meetings: every newly closed set and every
//! generated set is intersected with the other side's closed layers and its
//! start set. A meeting at `g_f + g_b` below the best one so far is kept.
//!
//! The search stops when the best meeting costs at most
//!
//! ```text
//! max(f_min(forward), f_min(backward), g_min(forward) + g_min(backward))
//! ```
//!
//! with `f = g + h` (just `g` for Dijkstra). With admissible heuristics each
//! term is a lower bound on any plan not met yet: take an optimal plan and the
//! longest prefix the forward side closed at optimal costs, and likewise the
//! longest suffix of the backward side. If they overlap or touch, a check has
//! seen the meeting. Otherwise the next state after the prefix is open forward
//! with `f ≤ C*`, the one before the suffix is open backward with `f ≤ C*`,
//! and their `g` values add up to at most `C*`. Counting each start set as
//! closed at cost 0 keeps this true before a side has expanded anything.

use sas_plus::OpId;

use crate::backend::{Backend, BackendResult};
use crate::model::frontier::Frontier;
use crate::model::{Direction, Model};
use crate::{Error, Solution};

/// The cheapest meeting of the two frontiers so far.
struct Meeting<S> {
    /// Plan cost through the meeting state, `g_forward + g_backward`.
    cost: u64,
    /// The singleton meeting state.
    state: S,
    /// Its cost in the forward frontier.
    g_forward: u64,
    /// Its cost in the backward frontier.
    g_backward: u64,
}

/// Called before each expansion with the direction about to expand and its
/// frontier. `SymBA*` uses it to install abstraction heuristics.
pub(crate) type Hook<'h, 'a, B> = dyn FnMut(&mut Frontier<'a, B>) -> Result<(), Error> + 'h;

impl<B: Backend> Model<B> {
    /// Runs `forward` and `backward` until they meet at optimal cost, calling
    /// `hook` before each expansion.
    ///
    /// Returns a plan in execution order, or `None` when the goal is not
    /// reachable. Optimal when both frontiers use admissible heuristics
    /// with [`crate::model::frontier::Strategy::Cheapest`].
    ///
    /// # Errors
    ///
    /// [`Error::OutOfMemory`] when the node table fills up, or any error of
    /// `hook`. An [`Error::BudgetExhausted`] from `hook` is filled in with
    /// the cheapest plan met so far and the current lower bound.
    pub(crate) fn bidirectional<'a>(
        &self,
        mut forward: Frontier<'a, B>,
        mut backward: Frontier<'a, B>,
        hook: &mut Hook<'_, 'a, B>,
    ) -> Result<Option<Solution>, Error> {
        let mut best: Option<Meeting<B::Set>> = None;
        while let Some(bound) = lower_bound(&forward, &backward) {
            if best.as_ref().is_some_and(|m| m.cost <= bound) {
                break;
            }

            let (this, other) = if pick_forward(&forward, &backward) {
                (&mut forward, &mut backward)
            } else {
                (&mut backward, &mut forward)
            };
            if let Err(error) = hook(this) {
                return Err(self.enrich(error, &forward, &backward, best)?);
            }

            let Some((g, states)) = this.pop()? else {
                continue;
            };

            self.meet(this, other, g, &states, &mut best)?;
            for (cost, generated) in this.expand(g, &states)? {
                self.meet(this, other, cost, &generated, &mut best)?;
            }
        }

        let Some(meeting) = best else {
            return Ok(None);
        };

        Ok(Some(self.solution(
            self.trace_meeting(&forward, &backward, meeting)?,
        )))
    }

    /// The plan in execution order through `meeting`.
    ///
    /// # Errors
    ///
    /// Out of memory in the backend.
    fn trace_meeting(
        &self,
        forward: &Frontier<'_, B>,
        backward: &Frontier<'_, B>,
        meeting: Meeting<B::Set>,
    ) -> BackendResult<Vec<OpId>> {
        let mut plan = self.trace_frontier(forward, meeting.state.clone(), meeting.g_forward)?;
        plan.reverse();
        plan.extend(self.trace_frontier(backward, meeting.state, meeting.g_backward)?);

        Ok(plan)
    }

    /// `error` from the hook, with [`Error::BudgetExhausted`] filled in: the
    /// plan through `best` and the termination bound, which no unmet plan
    /// undercuts (so the optimum is at least the smaller of the two).
    ///
    /// # Errors
    ///
    /// Out of memory in the backend while tracing the plan.
    fn enrich(
        &self,
        error: Error,
        forward: &Frontier<'_, B>,
        backward: &Frontier<'_, B>,
        best: Option<Meeting<B::Set>>,
    ) -> Result<Error, Error> {
        if !matches!(error, Error::BudgetExhausted { .. }) {
            return Ok(error);
        }

        let bound = lower_bound(forward, backward).unwrap_or(u64::MAX);
        let lower_bound = best.as_ref().map_or(bound, |m| m.cost.min(bound));
        let best = match best {
            Some(meeting) => Some(self.solution(self.trace_meeting(forward, backward, meeting)?)),
            None => None,
        };

        Ok(Error::BudgetExhausted { best, lower_bound })
    }

    /// Intersects `states`, found at cost `g` by `this`, with the closed
    /// layers and the start set of `other`, and records a cheaper meeting in
    /// `best`.
    ///
    /// # Errors
    ///
    /// Out of memory in the backend.
    fn meet(
        &self,
        this: &Frontier<'_, B>,
        other: &Frontier<'_, B>,
        g: u64,
        states: &B::Set,
        best: &mut Option<Meeting<B::Set>>,
    ) -> BackendResult<()> {
        let b = &self.backend;
        let start = core::iter::once((&0, other.start()));
        for (&other_g, layer) in start.chain(other.closed()) {
            let cost = g.saturating_add(other_g);
            if best.as_ref().is_some_and(|m| m.cost <= cost) {
                continue;
            }

            let common = b.and(states, layer)?;
            if b.is_empty(&common) {
                continue;
            }

            let (g_forward, g_backward) = match this.direction() {
                Direction::Forward => (g, other_g),
                Direction::Backward => (other_g, g),
            };
            *best = Some(Meeting {
                cost,
                state: self.pick_state(&common)?,
                g_forward,
                g_backward,
            });
        }

        Ok(())
    }
}

/// The termination bound of the module docs, or `None` when both sides are
/// exhausted.
fn lower_bound<B: Backend>(forward: &Frontier<'_, B>, backward: &Frontier<'_, B>) -> Option<u64> {
    let f = forward
        .min_priority()
        .into_iter()
        .chain(backward.min_priority());
    let g_sum = forward
        .min_g()
        .zip(backward.min_g())
        .map(|(gf, gb)| gf.saturating_add(gb));

    f.chain(g_sum).max()
}

/// Whether to expand forward next: the side with the smaller next bucket,
/// or the only side with anything left.
fn pick_forward<B: Backend>(forward: &Frontier<'_, B>, backward: &Frontier<'_, B>) -> bool {
    if backward.is_exhausted() {
        return true;
    }

    !forward.is_exhausted() && forward.next_size() <= backward.next_size()
}
