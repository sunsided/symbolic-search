//! The public entry point: a task compiled to BDDs, and its searches.

use core::sync::atomic::{AtomicU64, Ordering};

use sas_plus::{Fact, OpId, State, Task, VarId};

use crate::Error;
use crate::backend::bcdd::Oxidd;
use crate::model::frontier::{Frontier, Strategy};
use crate::model::heuristic::Partition;
use crate::model::{Direction, Model};
use crate::order::VariableOrder;

/// Resource limits and encoding options of a [`SymbolicTask`].
///
/// Build one with struct update syntax, so new fields do not break callers:
///
/// ```
/// use symbolic_search::Config;
///
/// let config = Config {
///     node_capacity: 1 << 16,
///     ..Config::default()
/// };
/// assert_eq!(config.cache_capacity, 1 << 18);
/// ```
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Config {
    /// Maximum number of live BDD nodes. The node table is reserved at this
    /// size when the task is compiled and never grows; searches that need
    /// more nodes fail with [`Error::OutOfMemory`]. A node takes about 16
    /// bytes plus its unique-table slot.
    pub node_capacity: usize,
    /// Number of entries of the operation cache. Larger caches avoid
    /// recomputation at the price of memory (a few dozen bytes per entry).
    pub cache_capacity: usize,
    /// Largest BDD, in nodes, that a merged transition-relation group may
    /// have. Operators of equal cost are merged into disjunctive groups up
    /// to this size; `0` keeps one relation per operator.
    pub merge_node_limit: usize,
    /// How the SAS+ variables are ordered in the diagram.
    pub variable_order: VariableOrder,
}

impl Default for Config {
    /// 2²⁰ nodes, 2¹⁸ cache entries, groups up to 10⁵ nodes (the `SymBA*`
    /// default), Gamer variable order.
    fn default() -> Self {
        Self {
            node_capacity: 1 << 20,
            cache_capacity: 1 << 18,
            merge_node_limit: 100_000,
            variable_order: VariableOrder::Gamer,
        }
    }
}

/// Source of [`SymbolicTask`] ids, which tie heuristics to their task.
static NEXT_TASK_ID: AtomicU64 = AtomicU64::new(0);

/// A SAS+ task compiled to binary decision diagrams.
///
/// Holds the encoding, the transition relations (per operator and merged
/// per cost), and the initial and goal sets. Searches reuse them and can be
/// run repeatedly. Plans are operator ids in execution order;
/// [`Task::validate_plan`] gives their cost.
///
/// | Search | Costs | Plan |
/// |---|---|---|
/// | [`forward_bfs`](Self::forward_bfs), [`backward_bfs`](Self::backward_bfs) | ignored | fewest operators |
/// | [`forward_dijkstra`](Self::forward_dijkstra), [`backward_dijkstra`](Self::backward_dijkstra), [`bidirectional_dijkstra`](Self::bidirectional_dijkstra) | used | cheapest |
/// | [`astar`](Self::astar) (`BDDA*`) | used | cheapest for an admissible heuristic |
/// | [`greedy`](Self::greedy) (GBFS) | used for tie breaking | any |
/// | [`symba`](Self::symba) (`SymBA*`) | used | cheapest |
pub struct SymbolicTask {
    /// The compiled model on the default backend.
    model: Model<Oxidd>,
    /// Unique id; [`Heuristic`]s remember it.
    id: u64,
}

/// A heuristic for [`SymbolicTask::astar`] and [`SymbolicTask::greedy`],
/// stored as one BDD per `h` value.
///
/// Built by [`SymbolicTask::pdb`] or [`SymbolicTask::heuristic`], and only
/// usable with the task that built it.
pub struct Heuristic {
    /// The partition, in the manager of the task with id `task`.
    partition: Partition<Oxidd>,
    /// Id of the [`SymbolicTask`] that built this heuristic.
    task: u64,
}

impl SymbolicTask {
    /// Compiles `task` into decision diagrams within the limits of `config`.
    ///
    /// # Errors
    ///
    /// - [`Error::InvalidTask`] when [`Task::validate`] fails.
    /// - [`Error::Axioms`] when the task has axioms or derived variables.
    /// - [`Error::TooManyBits`] when the encoding needs more than `u32::MAX`
    ///   diagram variables.
    /// - [`Error::OutOfMemory`] when the relations do not fit into
    ///   [`Config::node_capacity`].
    pub fn new(task: &Task, config: Config) -> Result<Self, Error> {
        Ok(Self {
            model: Model::new(task, &config)?,
            id: NEXT_TASK_ID.fetch_add(1, Ordering::Relaxed),
        })
    }

    /// Symbolic forward breadth-first search from the initial state.
    ///
    /// Returns a plan with the fewest operators (operator costs are ignored),
    /// as operator ids in execution order, or `None` when no goal state is
    /// reachable. The plan is empty when the initial state is a goal state.
    ///
    /// # Errors
    ///
    /// [`Error::OutOfMemory`] when the node table fills up.
    pub fn forward_bfs(&self) -> Result<Option<Vec<OpId>>, Error> {
        self.model.bfs(Direction::Forward)
    }

    /// Symbolic backward breadth-first search from the goal states.
    ///
    /// Same contract as [`SymbolicTask::forward_bfs`]; the plan is again in
    /// execution order. Both find plans of equal length, but not necessarily
    /// the same plan.
    ///
    /// # Errors
    ///
    /// [`Error::OutOfMemory`] when the node table fills up.
    pub fn backward_bfs(&self) -> Result<Option<Vec<OpId>>, Error> {
        self.model.bfs(Direction::Backward)
    }

    /// Symbolic Dijkstra (uniform-cost search) forward from the initial state.
    ///
    /// Returns a cheapest plan, or `None` when no goal state is reachable.
    /// Zero-cost operators are fine.
    ///
    /// # Errors
    ///
    /// [`Error::OutOfMemory`] when the node table fills up.
    ///
    /// # Examples
    ///
    /// ```
    /// use sas_plus::{Effect, Fact, Operator, State, Task, Variable};
    /// use symbolic_search::{Config, SymbolicTask};
    ///
    /// // A cheap detour (two steps of cost 1) beats a direct step of cost 5.
    /// let op = |name: &str, from, to, cost| Operator {
    ///     name: name.into(),
    ///     pre: vec![Fact::new(0, from)],
    ///     eff: vec![Effect::new(Fact::new(0, to))],
    ///     cost,
    /// };
    /// let task = Task {
    ///     variables: vec![Variable::new("at", 3)],
    ///     operators: vec![op("direct", 0, 2, 5), op("a", 0, 1, 1), op("b", 1, 2, 1)],
    ///     initial: State(vec![0]),
    ///     goal: vec![Fact::new(0, 2)],
    ///     axioms: vec![],
    /// };
    /// let config = Config { node_capacity: 1 << 10, cache_capacity: 1 << 8, ..Config::default() };
    ///
    /// let symbolic = SymbolicTask::new(&task, config)?;
    /// let plan = symbolic.forward_dijkstra()?.expect("solvable");
    /// assert_eq!(task.validate_plan(&plan), Some(2));
    /// # Ok::<(), symbolic_search::Error>(())
    /// ```
    pub fn forward_dijkstra(&self) -> Result<Option<Vec<OpId>>, Error> {
        self.model
            .best_first(Direction::Forward, self.blind(), Strategy::Cheapest)
    }

    /// Symbolic Dijkstra backward from the goal states.
    ///
    /// Same contract as [`SymbolicTask::forward_dijkstra`].
    ///
    /// # Errors
    ///
    /// [`Error::OutOfMemory`] when the node table fills up.
    pub fn backward_dijkstra(&self) -> Result<Option<Vec<OpId>>, Error> {
        self.model
            .best_first(Direction::Backward, self.blind(), Strategy::Cheapest)
    }

    /// Bidirectional symbolic Dijkstra: forward and backward uniform-cost
    /// search that stop once the cheapest meeting is proven optimal. Each
    /// step expands the side whose next set has fewer BDD nodes.
    ///
    /// Same contract as [`SymbolicTask::forward_dijkstra`].
    ///
    /// # Errors
    ///
    /// [`Error::OutOfMemory`] when the node table fills up.
    ///
    /// # Examples
    ///
    /// ```
    /// use sas_plus::{Effect, Fact, Operator, State, Task, Variable};
    /// use symbolic_search::{Config, SymbolicTask};
    ///
    /// let op = |name: &str, from, to, cost| Operator {
    ///     name: name.into(),
    ///     pre: vec![Fact::new(0, from)],
    ///     eff: vec![Effect::new(Fact::new(0, to))],
    ///     cost,
    /// };
    /// let task = Task {
    ///     variables: vec![Variable::new("at", 3)],
    ///     operators: vec![op("direct", 0, 2, 5), op("a", 0, 1, 1), op("b", 1, 2, 1)],
    ///     initial: State(vec![0]),
    ///     goal: vec![Fact::new(0, 2)],
    ///     axioms: vec![],
    /// };
    /// let config = Config { node_capacity: 1 << 10, cache_capacity: 1 << 8, ..Config::default() };
    /// let symbolic = SymbolicTask::new(&task, config)?;
    ///
    /// let plan = symbolic.bidirectional_dijkstra()?.expect("solvable");
    /// assert_eq!(plan, [1, 2]);
    /// # Ok::<(), symbolic_search::Error>(())
    /// ```
    pub fn bidirectional_dijkstra(&self) -> Result<Option<Vec<OpId>>, Error> {
        let (model, b) = (&self.model, self.model.backend());
        let side = |direction| {
            Frontier::new(
                b,
                model.concrete(),
                direction,
                Partition::zero(b),
                Strategy::Cheapest,
            )
        };
        let (forward, backward) = (side(Direction::Forward)?, side(Direction::Backward)?);

        model.bidirectional(forward, backward, &mut |_| Ok(()))
    }

    /// `BDDA*`: symbolic A* forward from the initial state, guided by
    /// `heuristic` (an estimate of the cost to the goal).
    ///
    /// Open sets are split by `h` value and expanded by smallest `g + h`,
    /// ties to the larger `g`. States are reopened when found more cheaply,
    /// so the plan is cheapest whenever `heuristic` is admissible, consistent
    /// or not. States the heuristic marks dead are pruned.
    ///
    /// # Errors
    ///
    /// - [`Error::ForeignHeuristic`] when `heuristic` belongs to another task.
    /// - [`Error::OutOfMemory`] when the node table fills up.
    ///
    /// # Examples
    ///
    /// ```
    /// use sas_plus::{Effect, Fact, Operator, State, Task, Variable};
    /// use symbolic_search::{Config, SymbolicTask};
    ///
    /// let inc = |from, to| Operator {
    ///     name: format!("inc{from}"),
    ///     pre: vec![Fact::new(0, from)],
    ///     eff: vec![Effect::new(Fact::new(0, to))],
    ///     cost: 2,
    /// };
    /// let task = Task {
    ///     variables: vec![Variable::new("counter", 3), Variable::new("noise", 2)],
    ///     operators: vec![inc(0, 1), inc(1, 2)],
    ///     initial: State(vec![0, 0]),
    ///     goal: vec![Fact::new(0, 2)],
    ///     axioms: vec![],
    /// };
    /// let config = Config { node_capacity: 1 << 10, cache_capacity: 1 << 8, ..Config::default() };
    /// let symbolic = SymbolicTask::new(&task, config)?;
    ///
    /// // The projection onto the counter is a perfect heuristic here.
    /// let pdb = symbolic.pdb(&[0])?;
    /// assert_eq!(symbolic.evaluate(&pdb, &task.initial)?, Some(4));
    /// let plan = symbolic.astar(&pdb)?.expect("solvable");
    /// assert_eq!(task.validate_plan(&plan), Some(4));
    /// # Ok::<(), symbolic_search::Error>(())
    /// ```
    pub fn astar(&self, heuristic: &Heuristic) -> Result<Option<Vec<OpId>>, Error> {
        let partition = self.own(heuristic)?;

        self.model
            .best_first(Direction::Forward, partition, Strategy::Cheapest)
    }

    /// Symbolic greedy best-first search forward: expands the open states
    /// with the smallest `h` first, ties to the smaller `g`, and never
    /// reopens a state.
    ///
    /// Returns some plan (not necessarily a cheap one), or `None` when no goal
    /// state is reachable through states `heuristic` keeps alive.
    ///
    /// # Errors
    ///
    /// - [`Error::ForeignHeuristic`] when `heuristic` belongs to another task.
    /// - [`Error::OutOfMemory`] when the node table fills up.
    ///
    /// # Examples
    ///
    /// ```
    /// use sas_plus::{Effect, Fact, Operator, State, Task, Variable};
    /// use symbolic_search::{Config, SymbolicTask};
    ///
    /// let op = |name: &str, from, to, cost| Operator {
    ///     name: name.into(),
    ///     pre: vec![Fact::new(0, from)],
    ///     eff: vec![Effect::new(Fact::new(0, to))],
    ///     cost,
    /// };
    /// let task = Task {
    ///     variables: vec![Variable::new("at", 3)],
    ///     operators: vec![op("direct", 0, 2, 5), op("a", 0, 1, 1), op("b", 1, 2, 1)],
    ///     initial: State(vec![0]),
    ///     goal: vec![Fact::new(0, 2)],
    ///     axioms: vec![],
    /// };
    /// let config = Config { node_capacity: 1 << 10, cache_capacity: 1 << 8, ..Config::default() };
    /// let symbolic = SymbolicTask::new(&task, config)?;
    ///
    /// // A misleading estimate: "a" looks far from the goal, so GBFS goes direct.
    /// let h = symbolic.heuristic(&[(vec![Fact::new(0, 1)], 9)])?;
    /// let plan = symbolic.greedy(&h)?.expect("solvable");
    /// assert_eq!(task.validate_plan(&plan), Some(5));
    /// # Ok::<(), symbolic_search::Error>(())
    /// ```
    pub fn greedy(&self, heuristic: &Heuristic) -> Result<Option<Vec<OpId>>, Error> {
        let partition = self.own(heuristic)?;

        self.model
            .best_first(Direction::Forward, partition, Strategy::Greedy)
    }

    /// `SymBA*`: bidirectional symbolic A* that switches from concrete to
    /// abstract search whenever a frontier grows past `frontier_node_limit`
    /// BDD nodes.
    ///
    /// Both directions start blind. When the side about to expand has a
    /// bigger next set than the limit, an abstract search in the opposite
    /// direction runs on a domain abstraction that drops one more variable
    /// (non-goal variables first). It expands at least its start, then stops
    /// once its own next set passes the same limit. Its perimeter becomes
    /// that side's heuristic (maximised with the previous one). The plan is
    /// always cheapest; the limit only trades concrete for abstract work.
    /// Small limits mean many short abstract searches with weak perimeters;
    /// `usize::MAX` gives bidirectional Dijkstra.
    ///
    /// # Errors
    ///
    /// [`Error::OutOfMemory`] when the node table fills up.
    ///
    /// # Examples
    ///
    /// ```
    /// use sas_plus::{Effect, Fact, Operator, State, Task, Variable};
    /// use symbolic_search::{Config, SymbolicTask};
    ///
    /// let op = |name: &str, from, to, cost| Operator {
    ///     name: name.into(),
    ///     pre: vec![Fact::new(0, from)],
    ///     eff: vec![Effect::new(Fact::new(0, to))],
    ///     cost,
    /// };
    /// let task = Task {
    ///     variables: vec![Variable::new("at", 3)],
    ///     operators: vec![op("direct", 0, 2, 5), op("a", 0, 1, 1), op("b", 1, 2, 1)],
    ///     initial: State(vec![0]),
    ///     goal: vec![Fact::new(0, 2)],
    ///     axioms: vec![],
    /// };
    /// let config = Config { node_capacity: 1 << 10, cache_capacity: 1 << 8, ..Config::default() };
    /// let symbolic = SymbolicTask::new(&task, config)?;
    ///
    /// for limit in [1, 64, usize::MAX] {
    ///     let plan = symbolic.symba(limit)?.expect("solvable");
    ///     assert_eq!(task.validate_plan(&plan), Some(2));
    /// }
    /// # Ok::<(), symbolic_search::Error>(())
    /// ```
    pub fn symba(&self, frontier_node_limit: usize) -> Result<Option<Vec<OpId>>, Error> {
        Ok(self.model.symba(frontier_node_limit)?.0)
    }

    /// The symbolic pattern database of `pattern`: the exact cost to the goal
    /// in the projection of the task onto the variables in `pattern`.
    ///
    /// States whose projection cannot reach the goal are dead ends.
    /// Conditional effects are projected exactly: an effect whose condition
    /// mentions a dropped variable may fire or not, but only in ways some
    /// concrete state allows. The heuristic is admissible and consistent.
    ///
    /// # Errors
    ///
    /// - [`Error::UnknownVariable`] when `pattern` names a variable the task
    ///   does not have.
    /// - [`Error::OutOfMemory`] when the node table fills up.
    pub fn pdb(&self, pattern: &[VarId]) -> Result<Heuristic, Error> {
        if let Some(&var) = pattern.iter().find(|&&v| v >= self.model.num_vars()) {
            return Err(Error::UnknownVariable(var));
        }

        Ok(self.wrap(self.model.pdb(pattern)?))
    }

    /// A heuristic from explicit estimates: each `(facts, h)` pair says that
    /// states satisfying all `facts` have at least cost `h` to the goal.
    ///
    /// A state gets the largest `h` among the pairs it satisfies, or 0 when
    /// it satisfies none. Admissibility is up to the caller; [`astar`]
    /// returns cheapest plans only for admissible estimates.
    ///
    /// [`astar`]: Self::astar
    ///
    /// # Errors
    ///
    /// - [`Error::InvalidFact`] when a fact names an unknown variable or a
    ///   value outside its domain.
    /// - [`Error::OutOfMemory`] when the node table fills up.
    pub fn heuristic(&self, estimates: &[(Vec<Fact>, u64)]) -> Result<Heuristic, Error> {
        let pairs = estimates
            .iter()
            .map(|(facts, h)| Ok((*h, self.model.facts(facts)?)))
            .collect::<Result<Vec<_>, Error>>()?;

        Ok(self.wrap(Partition::from_overlapping(
            self.model.backend(),
            pairs,
            Some(0),
        )?))
    }

    /// The value of `heuristic` in `state`, or `None` for a dead end.
    ///
    /// # Errors
    ///
    /// - [`Error::ForeignHeuristic`] when `heuristic` belongs to another task.
    /// - [`Error::InvalidFact`] when `state` has a value outside a domain or
    ///   the wrong number of variables (reported for the first offending
    ///   variable, or the first missing one).
    /// - [`Error::OutOfMemory`] when the node table fills up.
    pub fn evaluate(&self, heuristic: &Heuristic, state: &State) -> Result<Option<u64>, Error> {
        let partition = self.own(heuristic)?;
        if state.0.len() != self.model.num_vars() {
            return Err(Error::InvalidFact(Fact::new(state.0.len(), 0)));
        }

        let facts: Vec<Fact> = state
            .0
            .iter()
            .enumerate()
            .map(|(var, &value)| Fact::new(var, value))
            .collect();
        let set = self.model.facts(&facts)?;

        Ok(partition.value(self.model.backend(), &set)?)
    }

    /// The blind heuristic in this task's manager.
    fn blind(&self) -> Partition<Oxidd> {
        Partition::zero(self.model.backend())
    }

    /// A copy of the partition of `heuristic`, checked to belong to this
    /// task.
    ///
    /// # Errors
    ///
    /// [`Error::ForeignHeuristic`] when it was built by another task.
    fn own(&self, heuristic: &Heuristic) -> Result<Partition<Oxidd>, Error> {
        if heuristic.task != self.id {
            return Err(Error::ForeignHeuristic);
        }

        Ok(heuristic.partition.clone())
    }

    /// Wraps `partition` as a heuristic of this task.
    fn wrap(&self, partition: Partition<Oxidd>) -> Heuristic {
        Heuristic {
            partition,
            task: self.id,
        }
    }
}
