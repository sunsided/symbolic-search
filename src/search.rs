//! The public entry point: a task compiled to BDDs, and its searches.

use core::sync::atomic::{AtomicU64, Ordering};
use core::time::Duration;
use std::sync::{Arc, Mutex};

use sas_plus::{Fact, OpId, State, Task, VarId};

use crate::Error;
use crate::backend::{Backend, Oxidd};
use crate::model::frontier::Strategy;
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
    /// size when the task is compiled and never grows. A node takes about 16
    /// bytes plus its unique-table slot. See
    /// [`Config::max_node_capacity`] for what happens when it is too small.
    pub node_capacity: usize,
    /// Largest node table an automatic retry may use. When compiling, or a
    /// search without a [`Heuristic`] argument, runs out of nodes, the task
    /// is compiled again with twice the node and cache capacity, up to this
    /// many nodes, and the search reruns on the larger manager (which later
    /// searches reuse). Only past this cap do they fail with
    /// [`Error::OutOfMemory`]. Set it to [`Config::node_capacity`] or below
    /// to turn retries off. Searches that take a [`Heuristic`] never retry,
    /// because the heuristic's BDDs live in the original manager.
    pub max_node_capacity: usize,
    /// Number of entries of the operation cache. Larger caches avoid
    /// recomputation at the price of memory (a few dozen bytes per entry).
    pub cache_capacity: usize,
    /// Largest BDD, in nodes, that a merged transition-relation group may
    /// have. Operators of equal cost are merged into disjunctive groups up
    /// to this size; `0` keeps one relation per operator. Ignored under
    /// [`Partitioning::Conjunctive`].
    pub merge_node_limit: usize,
    /// How the SAS+ variables are ordered in the diagram.
    pub variable_order: VariableOrder,
    /// How transition relations are split for search steps.
    pub partitioning: Partitioning,
}

impl Config {
    /// Capacities sized for `task`, everything else as in
    /// [`Config::default`].
    ///
    /// The node table gets `2⁹ · bits + 2⁶ · bits · operators` nodes,
    /// rounded up to a power of two and clamped to `2¹²..=2²²`, where
    /// `bits` is the number of state bits; the cache gets a quarter of
    /// that. Relations grow with bits per operator, and search sets with the
    /// number of bits. The estimate is a starting point:
    /// [`Config::max_node_capacity`] (16 times the estimate, at least 2²³)
    /// covers tasks that need more.
    ///
    /// # Examples
    ///
    /// ```
    /// use sas_plus::{Effect, Fact, Operator, State, Task, Variable};
    /// use symbolic_search::{Config, SymbolicTask};
    ///
    /// let task = Task {
    ///     variables: vec![Variable::new("x", 4)],
    ///     operators: vec![Operator {
    ///         name: "set".into(),
    ///         pre: vec![],
    ///         eff: vec![Effect::new(Fact::new(0, 3))],
    ///         cost: 1,
    ///     }],
    ///     initial: State(vec![0]),
    ///     goal: vec![Fact::new(0, 3)],
    ///     axioms: vec![],
    /// };
    /// let config = Config::for_task(&task);
    /// assert_eq!(config.node_capacity, 1 << 12);
    ///
    /// let solution = SymbolicTask::new(&task, config)?.forward_bfs()?;
    /// assert_eq!(solution.map(|s| s.plan), Some(vec![0]));
    /// # Ok::<(), symbolic_search::Error>(())
    /// ```
    #[must_use]
    pub fn for_task(task: &Task) -> Self {
        let bits: usize = task
            .variables
            .iter()
            .map(|var| u32::BITS - var.domain_size.saturating_sub(1).leading_zeros())
            .map(|width| usize::try_from(width).unwrap_or(usize::MAX))
            .fold(0, usize::saturating_add);
        let estimate = bits.saturating_mul(1 << 9).saturating_add(
            bits.saturating_mul(task.operators.len())
                .saturating_mul(1 << 6),
        );
        let nodes = estimate.clamp(1 << 12, 1 << 22).next_power_of_two();

        Self {
            node_capacity: nodes,
            cache_capacity: nodes / 4,
            max_node_capacity: nodes.saturating_mul(16).max(1 << 23),
            ..Self::default()
        }
    }

    /// `self` with node and cache capacity multiplied by `factor`.
    fn scaled(self, factor: usize) -> Self {
        Self {
            node_capacity: self.node_capacity.saturating_mul(factor),
            cache_capacity: self.cache_capacity.saturating_mul(factor),
            ..self
        }
    }
}

/// How the transition relation is split for image and preimage.
///
/// Both give exactly the same images, preimages and therefore plans; they
/// differ in BDD sizes and speed.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Partitioning {
    /// Disjunctive groups: operators of equal cost are merged into one BDD
    /// each, up to [`Config::merge_node_limit`] nodes, and each group's image
    /// is one relational product.
    #[default]
    Disjunctive,
    /// Conjunctive clusters: one relation per operator (no merging), split
    /// into one conjunct per affected variable plus the remaining
    /// preconditions. Consecutive conjuncts in the quantification
    /// `schedule` are clustered while a cluster has at most
    /// `cluster_node_limit` nodes; images and preimages quantify each
    /// variable right after its last cluster. `0` keeps one cluster per
    /// conjunct, `usize::MAX` one cluster per operator.
    ///
    /// [`Config::merge_node_limit`] is ignored, and abstractions
    /// (PDBs, `SymBA*` perimeters) step with the monolithic operator
    /// relations.
    ///
    /// ```
    /// use symbolic_search::{Config, Partitioning, Schedule};
    ///
    /// let config = Config {
    ///     partitioning: Partitioning::Conjunctive {
    ///         cluster_node_limit: 0,
    ///         schedule: Schedule::Iwls95,
    ///     },
    ///     ..Config::default()
    /// };
    /// assert_ne!(config, Config::default());
    /// ```
    Conjunctive {
        /// Largest BDD, in nodes, of a cluster of conjuncts.
        cluster_node_limit: usize,
        /// The order in which conjuncts enter the product.
        schedule: Schedule,
    },
}

/// The quantification schedule of [`Partitioning::Conjunctive`]: the order
/// in which an operator's conjuncts are conjoined (and then clustered).
///
/// Both give the same images; they differ in intermediate BDD sizes.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Schedule {
    /// Most variables retired first: the conjunct after which the most
    /// affected variables can be quantified. The default: on the gripper
    /// fixtures its summed peak intermediate product is 4390 nodes against
    /// 4404 for IWLS95.
    #[default]
    Greedy,
    /// The IWLS95 benefit function of Ranjan et al. (1995): a weighted sum
    /// of the retired fraction of the conjunct's support, its share of the
    /// variables still to quantify, the next-state variables it introduces,
    /// and how deep in the variable order its variables sit.
    Iwls95,
}

impl Default for Config {
    /// 2²⁰ nodes, 2¹⁸ cache entries, retries up to 2²³ nodes, groups up to
    /// 10⁵ nodes (the `SymBA*` default), Gamer variable order, disjunctive
    /// partitioning.
    fn default() -> Self {
        Self {
            node_capacity: 1 << 20,
            cache_capacity: 1 << 18,
            max_node_capacity: 1 << 23,
            merge_node_limit: 100_000,
            variable_order: VariableOrder::Gamer,
            partitioning: Partitioning::Disjunctive,
        }
    }
}

/// A plan in execution order and its cost.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Solution {
    /// Operator ids in execution order. Empty when the initial state is a
    /// goal state.
    pub plan: Vec<OpId>,
    /// Sum of the plan's operator costs, as [`Task::validate_plan`] gives
    /// it.
    pub cost: u64,
}

/// Parameters of [`SymbolicTask::symba_with`].
///
/// Build one with struct update syntax, so new fields do not break callers:
///
/// ```
/// use core::time::Duration;
/// use symbolic_search::SymbaConfig;
///
/// let config = SymbaConfig {
///     abstract_slice: 4,
///     time_limit: Some(Duration::from_secs(60)),
///     ..SymbaConfig::default()
/// };
/// assert_eq!(config.max_expansions, None);
/// ```
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SymbaConfig {
    /// Size, in BDD nodes, of the next concrete bucket above which the side
    /// about to expand gets abstract work first.
    pub frontier_node_limit: usize,
    /// Size, in BDD nodes, of the next abstract bucket above which an
    /// abstract search counts as too hard: it is frozen, its perimeter is
    /// kept for good, and the next coarser abstraction starts.
    pub abstract_node_limit: usize,
    /// Abstract expansions per piece of abstract work. After that many, the
    /// abstract search pauses, its current perimeter is installed, and it
    /// resumes where it stopped the next time its side needs abstract work.
    /// `usize::MAX` runs each abstract search until it is too hard or
    /// exhausted; `0` counts as 1.
    pub abstract_slice: usize,
    /// Maximum number of expansions, concrete and abstract together;
    /// `None` for no limit.
    pub max_expansions: Option<u64>,
    /// Wall-clock limit for the whole search, checked before every
    /// expansion; `None` for no limit.
    pub time_limit: Option<Duration>,
    /// How each level of the abstraction hierarchy coarsens the last.
    pub hierarchy: AbstractionHierarchy,
}

/// The abstraction hierarchy of [`SymbolicTask::symba_with`]: how each
/// level coarsens the previous one. Variables are visited non-goal first,
/// from the last id down, and every level is the exact induced abstraction
/// of a projection of the state bits.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum AbstractionHierarchy {
    /// Each level drops one more variable (a pattern database hierarchy).
    #[default]
    DropVariables,
    /// Each level drops one more bit of one variable, low bits first, which
    /// merges that variable's values pairwise: a shrink step in the spirit
    /// of merge-and-shrink, without bisimulation. One round drops the
    /// lowest remaining bit of every non-goal variable, then the goal
    /// variables follow, so all variables coarsen evenly.
    DropBits,
}

impl Default for SymbaConfig {
    /// Concrete and abstract node limits of 10⁴ nodes, slices of 8 abstract
    /// expansions, no budget.
    fn default() -> Self {
        Self {
            frontier_node_limit: 10_000,
            abstract_node_limit: 10_000,
            abstract_slice: 8,
            max_expansions: None,
            time_limit: None,
            hierarchy: AbstractionHierarchy::DropVariables,
        }
    }
}

/// `config` with doubled capacities, or `None` past
/// [`Config::max_node_capacity`].
fn grow(config: Config) -> Option<Config> {
    let grown = config.scaled(2);

    (grown.node_capacity <= config.max_node_capacity && grown != config).then_some(grown)
}

/// Compiles `task` with `config`, doubling the capacities on
/// [`Error::OutOfMemory`] up to [`Config::max_node_capacity`]. Returns the
/// configuration that fit.
///
/// # Errors
///
/// The errors of [`SymbolicTask::new`].
fn compile_growing<B: Backend>(
    task: &Task,
    mut config: Config,
) -> Result<(Config, Model<B>), Error> {
    loop {
        match Model::new(task, &config) {
            Err(Error::OutOfMemory) => config = grow(config).ok_or(Error::OutOfMemory)?,
            result => return Ok((config, result?)),
        }
    }
}

/// Source of [`SymbolicTask`] ids, which tie heuristics to their task.
static NEXT_TASK_ID: AtomicU64 = AtomicU64::new(0);

/// A SAS+ task compiled to binary decision diagrams.
///
/// Holds the encoding, the transition relations (per operator, plus merged
/// per cost or split into conjunctive clusters, see [`Partitioning`]), and
/// the initial and goal sets. Searches reuse them and can be run
/// repeatedly. Plans are operator ids in execution order;
/// [`Task::validate_plan`] gives their cost.
///
/// | Search | Costs | Plan |
/// |---|---|---|
/// | [`forward_bfs`](Self::forward_bfs), [`backward_bfs`](Self::backward_bfs) | ignored | fewest operators |
/// | [`forward_dijkstra`](Self::forward_dijkstra), [`backward_dijkstra`](Self::backward_dijkstra), [`bidirectional_dijkstra`](Self::bidirectional_dijkstra) | used | cheapest |
/// | [`astar`](Self::astar) (`BDDA*`) | used | cheapest for an admissible heuristic |
/// | [`greedy`](Self::greedy) (GBFS) | used for tie breaking | any |
/// | [`symba`](Self::symba), [`symba_with`](Self::symba_with) (`SymBA*`) | used | cheapest (or [`Error::BudgetExhausted`]) |
pub struct SymbolicTask<B: Backend = Oxidd> {
    /// The compiled model on backend `B`, in the manager that
    /// [`Heuristic`]s of this task live in.
    model: Model<B>,
    /// Unique id; [`Heuristic`]s remember it.
    id: u64,
    /// The task, kept for recompiling with a larger manager.
    task: Task,
    /// The configuration [`SymbolicTask::model`] was compiled with.
    config: Config,
    /// The largest model an out-of-memory retry built so far; searches
    /// without heuristics start there.
    grown: Mutex<Option<Arc<Grown<B>>>>,
}

/// A model compiled again with a larger manager after running out of nodes.
struct Grown<B: Backend> {
    /// Its configuration.
    config: Config,
    /// The model.
    model: Model<B>,
}

/// A heuristic for [`SymbolicTask::astar`] and [`SymbolicTask::greedy`],
/// stored as one BDD per `h` value.
///
/// Built by [`SymbolicTask::pdb`] or [`SymbolicTask::heuristic`], and only
/// usable with the task that built it.
pub struct Heuristic<B: Backend = Oxidd> {
    /// The partition, in the manager of the task with id `task`.
    partition: Partition<B>,
    /// Id of the [`SymbolicTask`] that built this heuristic.
    task: u64,
}

impl SymbolicTask<Oxidd> {
    /// Compiles `task` into decision diagrams on the default backend
    /// ([`Oxidd`]) within the limits of `config`.
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
        Self::with_backend(task, config)
    }
}

impl<B: Backend> SymbolicTask<B> {
    /// Compiles `task` into decision diagrams on backend `B`; see
    /// [`SymbolicTask::new`] for the default backend.
    ///
    /// # Errors
    ///
    /// As [`SymbolicTask::new`].
    ///
    /// # Examples
    ///
    /// ```
    /// use sas_plus::{Effect, Fact, Operator, State, Task, Variable};
    /// use symbolic_search::backend::{Backend, Oxidd};
    /// use symbolic_search::{Config, Error, Solution, SymbolicTask};
    ///
    /// /// Works on any backend.
    /// fn solve<B: Backend>(task: &Task) -> Result<Option<Solution>, Error> {
    ///     SymbolicTask::<B>::with_backend(task, Config::for_task(task))?.bidirectional_dijkstra()
    /// }
    ///
    /// let task = Task {
    ///     variables: vec![Variable::new("x", 2)],
    ///     operators: vec![Operator {
    ///         name: "flip".into(),
    ///         pre: vec![Fact::new(0, 0)],
    ///         eff: vec![Effect::new(Fact::new(0, 1))],
    ///         cost: 3,
    ///     }],
    ///     initial: State(vec![0]),
    ///     goal: vec![Fact::new(0, 1)],
    ///     axioms: vec![],
    /// };
    /// assert_eq!(solve::<Oxidd>(&task)?.map(|s| s.cost), Some(3));
    /// # Ok::<(), Error>(())
    /// ```
    pub fn with_backend(task: &Task, config: Config) -> Result<Self, Error> {
        let (config, model) = compile_growing(task, config)?;

        Ok(Self {
            model,
            id: NEXT_TASK_ID.fetch_add(1, Ordering::Relaxed),
            task: task.clone(),
            config,
            grown: Mutex::new(None),
        })
    }

    /// The node capacity the searches without a heuristic start from: the
    /// configured one, or the larger one an earlier retry grew to.
    #[must_use]
    pub fn node_capacity(&self) -> usize {
        self.grown_model()
            .map_or(self.config.node_capacity, |g| g.config.node_capacity)
    }

    /// Runs `search` on the largest model so far, and on running out of
    /// nodes, on models with doubled capacity up to
    /// [`Config::max_node_capacity`]. Remembers the largest model built.
    ///
    /// # Errors
    ///
    /// Any error of `search`; [`Error::OutOfMemory`] only past the cap.
    fn retrying<T>(&self, search: impl Fn(&Model<B>) -> Result<T, Error>) -> Result<T, Error> {
        let grown = self.grown_model();
        let (mut config, result) = match &grown {
            Some(g) => (g.config, search(&g.model)),
            None => (self.config, search(&self.model)),
        };
        if !matches!(result, Err(Error::OutOfMemory)) {
            return result;
        }

        loop {
            config = grow(config).ok_or(Error::OutOfMemory)?;
            let (grown_config, model) = compile_growing(&self.task, config)?;
            config = grown_config;
            let result = search(&model);
            self.remember(Grown { config, model });
            if !matches!(result, Err(Error::OutOfMemory)) {
                return result;
            }
        }
    }

    /// The model of the last retry, if any.
    fn grown_model(&self) -> Option<Arc<Grown<B>>> {
        self.grown
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
            .clone()
    }

    /// Keeps `grown` for later searches when it is larger than the one kept.
    fn remember(&self, grown: Grown<B>) {
        let mut slot = self
            .grown
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        if slot
            .as_ref()
            .is_none_or(|g| g.config.node_capacity < grown.config.node_capacity)
        {
            *slot = Some(Arc::new(grown));
        }
    }

    /// Symbolic forward breadth-first search from the initial state.
    ///
    /// Returns a plan with the fewest operators (operator costs are ignored
    /// in the search; the solution still reports the plan's cost), or `None` when no goal state is
    /// reachable. The plan is empty when the initial state is a goal state.
    ///
    /// # Errors
    ///
    /// [`Error::OutOfMemory`] when the node table fills up.
    pub fn forward_bfs(&self) -> Result<Option<Solution>, Error> {
        self.retrying(|model| model.bfs(Direction::Forward))
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
    pub fn backward_bfs(&self) -> Result<Option<Solution>, Error> {
        self.retrying(|model| model.bfs(Direction::Backward))
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
    /// let solution = symbolic.forward_dijkstra()?.expect("solvable");
    /// assert_eq!(task.validate_plan(&solution.plan), Some(2));
    /// assert_eq!(solution.cost, 2);
    /// # Ok::<(), symbolic_search::Error>(())
    /// ```
    pub fn forward_dijkstra(&self) -> Result<Option<Solution>, Error> {
        self.retrying(|model| model.dijkstra(Direction::Forward))
    }

    /// Symbolic Dijkstra backward from the goal states.
    ///
    /// Same contract as [`SymbolicTask::forward_dijkstra`].
    ///
    /// # Errors
    ///
    /// [`Error::OutOfMemory`] when the node table fills up.
    pub fn backward_dijkstra(&self) -> Result<Option<Solution>, Error> {
        self.retrying(|model| model.dijkstra(Direction::Backward))
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
    /// let solution = symbolic.bidirectional_dijkstra()?.expect("solvable");
    /// assert_eq!(solution.plan, [1, 2]);
    /// assert_eq!(solution.cost, 2);
    /// # Ok::<(), symbolic_search::Error>(())
    /// ```
    pub fn bidirectional_dijkstra(&self) -> Result<Option<Solution>, Error> {
        self.retrying(Model::bidirectional_dijkstra)
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
    /// let solution = symbolic.astar(&pdb)?.expect("solvable");
    /// assert_eq!(task.validate_plan(&solution.plan), Some(4));
    /// assert_eq!(solution.cost, 4);
    /// # Ok::<(), symbolic_search::Error>(())
    /// ```
    pub fn astar(&self, heuristic: &Heuristic<B>) -> Result<Option<Solution>, Error> {
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
    /// let solution = symbolic.greedy(&h)?.expect("solvable");
    /// assert_eq!(task.validate_plan(&solution.plan), Some(5));
    /// assert_eq!(solution.cost, 5);
    /// # Ok::<(), symbolic_search::Error>(())
    /// ```
    pub fn greedy(&self, heuristic: &Heuristic<B>) -> Result<Option<Solution>, Error> {
        let partition = self.own(heuristic)?;

        self.model
            .best_first(Direction::Forward, partition, Strategy::Greedy)
    }

    /// `SymBA*` with the same node limit for concrete frontiers and abstract
    /// searches, abstract searches run to that limit in one go, and no
    /// budget. Shorthand for [`SymbolicTask::symba_with`]; `usize::MAX`
    /// gives bidirectional Dijkstra.
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
    ///     let solution = symbolic.symba(limit)?.expect("solvable");
    ///     assert_eq!(solution.cost, 2);
    /// }
    /// # Ok::<(), symbolic_search::Error>(())
    /// ```
    pub fn symba(&self, node_limit: usize) -> Result<Option<Solution>, Error> {
        self.symba_with(&SymbaConfig {
            frontier_node_limit: node_limit,
            abstract_node_limit: node_limit,
            abstract_slice: usize::MAX,
            max_expansions: None,
            time_limit: None,
            hierarchy: AbstractionHierarchy::DropVariables,
        })
    }

    /// `SymBA*`: bidirectional symbolic A* that does abstract work whenever
    /// a frontier grows past [`SymbaConfig::frontier_node_limit`] BDD nodes.
    ///
    /// Both directions start blind. Each side owns a hierarchy of domain
    /// abstractions that drop one more variable per level (non-goal
    /// variables first) and keeps one abstract search, in the opposite
    /// direction, alive at a time. When the side about to expand has a
    /// bigger next bucket than the limit, its abstract search runs for
    /// [`SymbaConfig::abstract_slice`] expansions (always at least its
    /// start), then pauses and is resumed next time. Its perimeter, maximised
    /// with those of frozen searches, becomes the side's heuristic. A search
    /// whose next bucket passes [`SymbaConfig::abstract_node_limit`] is
    /// frozen and the next level starts; an exhausted one ends the side's
    /// abstract work, since coarser levels give weaker bounds. The plan is
    /// always cheapest; the limits only trade concrete for abstract work.
    ///
    /// # Errors
    ///
    /// - [`Error::BudgetExhausted`] when [`SymbaConfig::max_expansions`] or
    ///   [`SymbaConfig::time_limit`] runs out first.
    /// - [`Error::OutOfMemory`] when the node table fills up.
    ///
    /// # Examples
    ///
    /// ```
    /// use sas_plus::{Effect, Fact, Operator, State, Task, Variable};
    /// use symbolic_search::{Config, Error, SymbaConfig, SymbolicTask};
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
    /// let symba = SymbaConfig { frontier_node_limit: 1, abstract_slice: 1, ..SymbaConfig::default() };
    /// let solution = symbolic.symba_with(&symba)?.expect("solvable");
    /// assert_eq!(task.validate_plan(&solution.plan), Some(2));
    /// assert_eq!(solution.cost, 2);
    ///
    /// let starved = SymbaConfig { max_expansions: Some(0), ..symba };
    /// let Err(Error::BudgetExhausted { best, lower_bound }) = symbolic.symba_with(&starved) else {
    ///     panic!("no expansion is allowed");
    /// };
    /// assert_eq!((best, lower_bound), (None, 0));
    /// # Ok::<(), symbolic_search::Error>(())
    /// ```
    pub fn symba_with(&self, config: &SymbaConfig) -> Result<Option<Solution>, Error> {
        self.retrying(|model| Ok(model.symba(config)?.0))
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
    pub fn pdb(&self, pattern: &[VarId]) -> Result<Heuristic<B>, Error> {
        if let Some(&var) = pattern.iter().find(|&&v| v >= self.model.num_vars()) {
            return Err(Error::UnknownVariable(var));
        }

        Ok(self.wrap(self.model.pdb(pattern)?))
    }

    /// The canonical heuristic of a PDB collection: one symbolic PDB per
    /// pattern (see [`SymbolicTask::pdb`]), combined by taking the maximum
    /// over every maximal set of pairwise additive patterns of the sum of
    /// their PDBs. Two patterns are additive when no operator has an effect
    /// on a variable of each. Admissible and consistent, and at least as
    /// informed as the best single PDB. No patterns give the blind
    /// heuristic.
    ///
    /// # Errors
    ///
    /// - [`Error::UnknownVariable`] when a pattern names a variable the task
    ///   does not have.
    /// - [`Error::OutOfMemory`] when the node table fills up.
    ///
    /// # Examples
    ///
    /// ```
    /// use sas_plus::{Effect, Fact, Operator, State, Task, Variable};
    /// use symbolic_search::{Config, SymbolicTask};
    ///
    /// // Two independent counters: their PDBs add up.
    /// let inc = |var, from, to| Operator {
    ///     name: format!("inc{var}_{from}"),
    ///     pre: vec![Fact::new(var, from)],
    ///     eff: vec![Effect::new(Fact::new(var, to))],
    ///     cost: 1,
    /// };
    /// let task = Task {
    ///     variables: vec![Variable::new("a", 3), Variable::new("b", 3)],
    ///     operators: vec![inc(0, 0, 1), inc(0, 1, 2), inc(1, 0, 1), inc(1, 1, 2)],
    ///     initial: State(vec![0, 0]),
    ///     goal: vec![Fact::new(0, 2), Fact::new(1, 2)],
    ///     axioms: vec![],
    /// };
    /// let config = Config { node_capacity: 1 << 12, cache_capacity: 1 << 8, ..Config::default() };
    /// let symbolic = SymbolicTask::new(&task, config)?;
    ///
    /// let canonical = symbolic.canonical_pdbs(&[vec![0], vec![1]])?;
    /// assert_eq!(symbolic.evaluate(&canonical, &task.initial)?, Some(4));
    /// let single = symbolic.pdb(&[0])?;
    /// assert_eq!(symbolic.evaluate(&single, &task.initial)?, Some(2));
    /// # Ok::<(), symbolic_search::Error>(())
    /// ```
    pub fn canonical_pdbs(&self, patterns: &[Vec<VarId>]) -> Result<Heuristic<B>, Error> {
        let num_vars = self.model.num_vars();
        if let Some(&var) = patterns.iter().flatten().find(|&&v| v >= num_vars) {
            return Err(Error::UnknownVariable(var));
        }

        Ok(self.wrap(self.model.canonical(patterns)?))
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
    pub fn heuristic(&self, estimates: &[(Vec<Fact>, u64)]) -> Result<Heuristic<B>, Error> {
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
    pub fn evaluate(&self, heuristic: &Heuristic<B>, state: &State) -> Result<Option<u64>, Error> {
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

    /// A copy of the partition of `heuristic`, checked to belong to this
    /// task.
    ///
    /// # Errors
    ///
    /// [`Error::ForeignHeuristic`] when it was built by another task.
    fn own(&self, heuristic: &Heuristic<B>) -> Result<Partition<B>, Error> {
        if heuristic.task != self.id {
            return Err(Error::ForeignHeuristic);
        }

        Ok(heuristic.partition.clone())
    }

    /// Wraps `partition` as a heuristic of this task.
    fn wrap(&self, partition: Partition<B>) -> Heuristic<B> {
        Heuristic {
            partition,
            task: self.id,
        }
    }
}
