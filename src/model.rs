//! The symbolic model of a task and the searches on it.
//!
//! ```mermaid
//! graph LR
//!     Task --> Encoding
//!     Task --> Operators[per-operator relations]
//!     Encoding --> Operators
//!     Operators --> Groups[merged groups per cost]
//!     Groups --> Space[concrete or projected space]
//!     Space --> Frontier[open and closed per g]
//!     Heuristic[h partition] --> Frontier
//!     Frontier --> Trace[walk closed layers back to a plan]
//!     Operators --> Trace
//! ```
//!
//! Unit-cost BFS lives here; the cost-aware searches live in the
//! submodules.

mod abstraction;
mod best_first;
mod bidirectional;
pub(crate) mod frontier;
pub(crate) mod heuristic;
pub(crate) mod space;
mod symba;

use sas_plus::{OpId, Task};

use crate::backend::{Backend, BackendResult, Capacity};
use crate::encoding::Encoding;
use crate::relation::{self, Relation};
use crate::{Config, Error, Partitioning};
use frontier::Frontier;
use space::Space;

/// Search direction of one symbolic step.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Direction {
    /// Successors via [`Relation::image`].
    Forward,
    /// Predecessors via [`Relation::preimage`].
    Backward,
}

impl Direction {
    /// The other direction; plan extraction walks against the search.
    pub(crate) fn reverse(self) -> Self {
        match self {
            Self::Forward => Self::Backward,
            Self::Backward => Self::Forward,
        }
    }
}

/// The layers of a finished breadth-first search that hit its target.
struct Layers<S> {
    /// `layers[i]` holds the states first reached after `i` steps. Disjoint;
    /// `layers[0]` is the start set.
    layers: Vec<S>,
    /// The part of the last layer that lies in the target set. Never empty.
    hit: S,
}

/// One step of [`Model::trace_frontier`].
struct BackStep<S> {
    /// The operator of the step.
    op: OpId,
    /// The singleton state the step leads to.
    state: S,
    /// Its cost in the frontier.
    g: u64,
    /// Index of the closed entry that holds it.
    entry: usize,
}

/// A SAS+ task compiled to decision diagrams on backend `B`.
pub(crate) struct Model<B: Backend> {
    /// The diagram manager that owns every set below.
    backend: B,
    /// Bit layout of the task's variables.
    encoding: Encoding,
    /// One relation per operator, indexed by operator id. Used for plan
    /// reconstruction.
    operators: Vec<Relation<B>>,
    /// The concrete state space with merged relation groups, used for
    /// search steps.
    concrete: Space<B>,
    /// Variables of goal facts, sorted; abstractions drop them last.
    goal_vars: Vec<sas_plus::VarId>,
}

impl<B: Backend> Model<B> {
    /// Compiles `task` into diagrams with the limits and options of `config`.
    ///
    /// # Errors
    ///
    /// - [`Error::InvalidTask`] when [`Task::validate`] fails.
    /// - [`Error::Axioms`] when the task has derived variables.
    /// - [`Error::TooManyBits`] when the encoding exceeds `u32::MAX` diagram variables.
    /// - [`Error::OutOfMemory`] when the diagrams do not fit into the
    ///   configured capacity.
    pub(crate) fn new(task: &Task, config: &Config) -> Result<Self, Error> {
        task.validate().map_err(Error::InvalidTask)?;
        if !task.axioms.is_empty() || task.variables.iter().any(sas_plus::Variable::is_derived) {
            return Err(Error::Axioms);
        }

        let encoding = Encoding::new(task, config.variable_order).ok_or(Error::TooManyBits)?;
        let capacity = Capacity {
            nodes: config.node_capacity,
            cache: config.cache_capacity,
        };
        let backend = B::new(encoding.num_diagram_vars(), capacity)?;

        let operators = task
            .operators
            .iter()
            .map(|operator| Relation::new(&backend, &encoding, operator))
            .collect::<BackendResult<Vec<_>>>()?;
        let groups = match config.partitioning {
            Partitioning::Disjunctive => {
                relation::partition(&backend, &encoding, &operators, config.merge_node_limit)?
            }
            Partitioning::Conjunctive { cluster_node_limit } => relation::conjunctive_groups(
                &backend,
                &encoding,
                &task.operators,
                cluster_node_limit,
            )?,
        };
        let initial = encoding.state(&backend, &task.initial)?;
        let goal = backend.and(
            &encoding.facts(&backend, &task.goal)?,
            &encoding.all_valid(&backend)?,
        )?;
        let mut goal_vars: Vec<_> = task.goal.iter().map(|fact| fact.var).collect();
        goal_vars.sort_unstable();
        goal_vars.dedup();

        Ok(Self {
            backend,
            encoding,
            operators,
            concrete: Space {
                groups,
                initial,
                goal,
            },
            goal_vars,
        })
    }

    /// The diagram manager.
    pub(crate) fn backend(&self) -> &B {
        &self.backend
    }

    /// The concrete state space.
    pub(crate) fn concrete(&self) -> &Space<B> {
        &self.concrete
    }

    /// Number of SAS+ variables.
    pub(crate) fn num_vars(&self) -> usize {
        self.encoding.num_vars()
    }

    /// The states that satisfy all `facts`, checked against the domains.
    ///
    /// # Errors
    ///
    /// - [`Error::InvalidFact`] for a fact with an unknown variable or a
    ///   value outside its domain.
    /// - [`Error::OutOfMemory`] when the node table fills up.
    pub(crate) fn facts(&self, facts: &[sas_plus::Fact]) -> Result<B::Set, Error> {
        if let Some(&fact) = facts.iter().find(|f| !self.encoding.holds_value(**f)) {
            return Err(Error::InvalidFact(fact));
        }

        Ok(self.encoding.facts(&self.backend, facts)?)
    }

    /// Unit-cost breadth-first search in `direction`, then plan extraction.
    ///
    /// Returns a shortest plan, or `None` when the goal is unreachable.
    ///
    /// # Errors
    ///
    /// [`Error::OutOfMemory`] when the node table fills up.
    pub(crate) fn bfs(&self, direction: Direction) -> Result<Option<Vec<OpId>>, Error> {
        let (start, target) = (
            self.concrete.start(direction),
            self.concrete.start(direction.reverse()),
        );
        let Some(found) = self.explore(start, target, direction)? else {
            return Ok(None);
        };

        let plan = self.trace(&found, direction.reverse())?;

        Ok(Some(orient(plan, direction)))
    }

    /// Expands layers from `start` until one meets `target` or no new state
    /// appears.
    ///
    /// # Errors
    ///
    /// Out of memory in the backend.
    fn explore(
        &self,
        start: &B::Set,
        target: &B::Set,
        direction: Direction,
    ) -> BackendResult<Option<Layers<B::Set>>> {
        let b = &self.backend;
        let mut reached = start.clone();
        let mut layers = vec![start.clone()];
        loop {
            let frontier = layers.last().expect("layers start with the start set");
            let hit = b.and(frontier, target)?;
            if !b.is_empty(&hit) {
                return Ok(Some(Layers { layers, hit }));
            }

            let fresh = b.and(&self.successors(frontier, direction)?, &b.not(&reached)?)?;
            if b.is_empty(&fresh) {
                return Ok(None);
            }

            reached = b.or(&reached, &fresh)?;
            layers.push(fresh);
        }
    }

    /// One step from `states` in `direction`, over all relation groups and
    /// ignoring costs.
    ///
    /// # Errors
    ///
    /// Out of memory in the backend.
    fn successors(&self, states: &B::Set, direction: Direction) -> BackendResult<B::Set> {
        self.concrete
            .groups
            .iter()
            .try_fold(self.backend.constant(false), |acc, group| {
                self.backend
                    .or(&acc, &group.step(&self.backend, states, direction)?)
            })
    }

    /// Walks from one state of `found.hit` back through the layers, stepping
    /// in `walk` direction, and returns the operators in walk order.
    ///
    /// # Errors
    ///
    /// Out of memory in the backend.
    fn trace(&self, found: &Layers<B::Set>, walk: Direction) -> BackendResult<Vec<OpId>> {
        let mut state = self.pick_state(&found.hit)?;
        let mut ops = Vec::with_capacity(found.layers.len() - 1);
        for layer in found.layers.iter().rev().skip(1) {
            let (op, previous) = self.step_into(&state, layer, walk)?;
            ops.push(op);
            state = previous;
        }

        Ok(ops)
    }

    /// Finds an operator that leads from the singleton `state` into `layer`
    /// in `walk` direction, and one state it reaches there.
    ///
    /// # Errors
    ///
    /// Out of memory in the backend.
    ///
    /// # Panics
    ///
    /// When no operator connects `state` to `layer`. BFS layering rules this
    /// out: every state of layer `i + 1` has a neighbour in layer `i`.
    fn step_into(
        &self,
        state: &B::Set,
        layer: &B::Set,
        walk: Direction,
    ) -> BackendResult<(OpId, B::Set)> {
        for (op, relation) in self.operators.iter().enumerate() {
            let reached = self
                .backend
                .and(&relation.step(&self.backend, state, walk)?, layer)?;
            if !self.backend.is_empty(&reached) {
                return Ok((op, self.pick_state(&reached)?));
            }
        }

        panic!("a BFS layer state has no neighbour in the previous layer");
    }

    /// Walks from `state`, found at cost `g` by `frontier`, back to its start
    /// and returns the operators in walk order (see
    /// [`orient`] for execution order).
    ///
    /// `state` must be a singleton that is either closed by `frontier` at
    /// cost `g`, or was generated at cost `g` by an expansion of it.
    ///
    /// # Errors
    ///
    /// Out of memory in the backend.
    pub(crate) fn trace_frontier(
        &self,
        frontier: &Frontier<'_, B>,
        state: B::Set,
        g: u64,
    ) -> BackendResult<Vec<OpId>> {
        let b = &self.backend;
        let (mut state, mut g, mut before) = (state, g, frontier.entries().len());
        let mut ops = Vec::new();
        while g > 0 || b.is_empty(&b.and(&state, frontier.start())?) {
            let step = self.step_back(frontier, &state, g, before)?;
            ops.push(step.op);
            (state, g, before) = (step.state, step.g, step.entry);
        }

        Ok(ops)
    }

    /// Finds an operator `o` and a closed entry before index `before` with
    /// cost `g − cost(o)` that `o` connects to the singleton `state` in the
    /// frontier's direction.
    ///
    /// # Errors
    ///
    /// Out of memory in the backend.
    ///
    /// # Panics
    ///
    /// When no such entry exists; the frontier invariant rules this out.
    fn step_back(
        &self,
        frontier: &Frontier<'_, B>,
        state: &B::Set,
        g: u64,
        before: usize,
    ) -> BackendResult<BackStep<B::Set>> {
        let b = &self.backend;
        let walk = frontier.direction().reverse();
        for (op, relation) in self.operators.iter().enumerate() {
            let Some(target) = g.checked_sub(relation.cost) else {
                continue;
            };
            let neighbours = relation.step(b, state, walk)?;
            if b.is_empty(&neighbours) {
                continue;
            }

            for (entry, (entry_g, layer)) in frontier.entries()[..before].iter().enumerate().rev() {
                if *entry_g != target {
                    continue;
                }

                let reached = b.and(&neighbours, layer)?;
                if !b.is_empty(&reached) {
                    let state = self.pick_state(&reached)?;
                    return Ok(BackStep {
                        op,
                        state,
                        g: target,
                        entry,
                    });
                }
            }
        }

        panic!("a closed state has no predecessor in an earlier layer");
    }

    /// The singleton set of one state of the non-empty set `states`.
    ///
    /// # Errors
    ///
    /// Out of memory in the backend.
    ///
    /// # Panics
    ///
    /// When `states` is empty.
    pub(crate) fn pick_state(&self, states: &B::Set) -> BackendResult<B::Set> {
        let assignment = self
            .backend
            .pick(states)
            .expect("picking from a non-empty set");

        self.encoding
            .state(&self.backend, &self.encoding.decode(&assignment))
    }
}

/// Puts a plan traced in the walk direction of a `direction` search into
/// execution order: a forward search is traced backwards from the goal.
fn orient(mut plan: Vec<OpId>, direction: Direction) -> Vec<OpId> {
    if direction == Direction::Forward {
        plan.reverse();
    }

    plan
}

#[cfg(test)]
mod tests {
    use sas_plus::{Effect, Fact, Operator, State, Variable};

    use super::*;
    use crate::SymbaConfig;
    use crate::backend::bcdd::Oxidd;

    /// A set in the test backend.
    type BddSet = <Oxidd as Backend>::Set;

    /// Tiny limits for unit tests.
    pub(crate) fn tiny_config() -> Config {
        Config {
            node_capacity: 1 << 12,
            cache_capacity: 1 << 8,
            ..Config::default()
        }
    }

    /// Backward search through an unconditional effect frees the variable's
    /// bits; the relation must still keep the preimage inside valid codes.
    #[test]
    fn backward_layers_stay_in_valid_codes() {
        let task = Task {
            variables: vec![Variable::new("x", 3)],
            operators: vec![Operator {
                name: "set".into(),
                pre: vec![],
                eff: vec![Effect::new(Fact::new(0, 2))],
                cost: 1,
            }],
            initial: State(vec![0]),
            goal: vec![Fact::new(0, 2)],
            axioms: vec![],
        };
        let model = Model::<Oxidd>::new(&task, &tiny_config()).unwrap();
        let b = &model.backend;

        let found = model
            .explore(
                &model.concrete.goal,
                &model.concrete.initial,
                Direction::Backward,
            )
            .unwrap()
            .expect("the initial state reaches the goal");
        let invalid = b.not(&model.encoding.all_valid(b).unwrap()).unwrap();

        for layer in &found.layers {
            assert!(b.is_empty(&b.and(layer, &invalid).unwrap()));
        }
    }

    /// A random task with conditional effects, a fixed seed.
    fn random(seed: u64) -> Task {
        let params = sas_plus::random::RandomTaskParams {
            max_domain: 5,
            max_conditions: 2,
            ..sas_plus::random::RandomTaskParams::default()
        };

        sas_plus::random::random_task(seed, params)
    }

    /// Merged groups step exactly like the union of their operators, both
    /// ways, from the set of all valid states.
    #[test]
    fn merged_groups_equal_operator_union() {
        for seed in 0..16 {
            let config = Config {
                merge_node_limit: usize::MAX,
                ..tiny_config()
            };
            let model = Model::<Oxidd>::new(&random(seed), &config).unwrap();
            let b = &model.backend;
            let all = model.encoding.all_valid(b).unwrap();
            assert!(model.concrete.groups.len() < model.operators.len());

            for direction in [Direction::Forward, Direction::Backward] {
                let union = model.operators.iter().fold(b.constant(false), |acc, op| {
                    b.or(&acc, &op.step(b, &all, direction).unwrap()).unwrap()
                });

                assert!(model.successors(&all, direction).unwrap() == union);
            }
        }
    }

    /// The union of the per-operator (monolithic) steps from `states`.
    fn operator_union(model: &Model<Oxidd>, states: &BddSet, direction: Direction) -> BddSet {
        let b = &model.backend;

        model.operators.iter().fold(b.constant(false), |acc, op| {
            b.or(&acc, &op.step(b, states, direction).unwrap()).unwrap()
        })
    }

    /// Start sets for step comparisons: all valid states, plus every layer
    /// of a forward and a backward BFS.
    fn probe_sets(model: &Model<Oxidd>) -> Vec<BddSet> {
        let b = &model.backend;
        let mut sets = vec![model.encoding.all_valid(b).unwrap()];
        for direction in [Direction::Forward, Direction::Backward] {
            let start = model.concrete.start(direction);
            let mut layer = start.clone();
            let mut reached = start.clone();
            while !b.is_empty(&layer) {
                sets.push(layer.clone());
                let next = model.successors(&layer, direction).unwrap();
                layer = b.and(&next, &b.not(&reached).unwrap()).unwrap();
                reached = b.or(&reached, &layer).unwrap();
            }
        }

        sets
    }

    /// Conjunctive clusters step exactly like the union of the monolithic
    /// operator relations, image and preimage, at cluster limits from one
    /// conjunct per cluster to one cluster per operator.
    #[test]
    fn conjunctive_clusters_equal_operator_union() {
        let mut split_with_conditions = 0;
        for limit in [0, 8, 64, usize::MAX] {
            for seed in 0..16 {
                let task = random(seed);
                let config = Config {
                    partitioning: Partitioning::Conjunctive {
                        cluster_node_limit: limit,
                    },
                    ..tiny_config()
                };
                let model = Model::<Oxidd>::new(&task, &config).unwrap();
                assert_eq!(model.concrete.groups.len(), model.operators.len());
                split_with_conditions +=
                    usize::from(limit == 0 && task.operators.iter().any(has_cross_condition));

                for states in probe_sets(&model) {
                    for direction in [Direction::Forward, Direction::Backward] {
                        let union = operator_union(&model, &states, direction);
                        assert!(model.successors(&states, direction).unwrap() == union);
                    }
                }
            }
        }

        assert!(
            split_with_conditions > 0,
            "no conditional cross-variable effect"
        );
    }

    /// Whether `operator` affects two variables and one of its effect
    /// conditions names another affected variable, so early quantification
    /// has to wait for a later conjunct.
    fn has_cross_condition(operator: &Operator) -> bool {
        let affected: Vec<_> = operator.eff.iter().map(|e| e.fact.var).collect();
        operator.eff.iter().any(|e| {
            e.conditions
                .iter()
                .any(|c| c.var != e.fact.var && affected.contains(&c.var))
        })
    }

    /// One cluster per conjunct gives more than one cluster for an operator
    /// with two effects, and the precondition on an untouched variable
    /// still filters.
    #[test]
    fn conjunctive_split_has_one_cluster_per_conjunct() {
        let task = Task {
            variables: vec![
                Variable::new("a", 2),
                Variable::new("b", 3),
                Variable::new("c", 2),
            ],
            operators: vec![Operator {
                name: "swap".into(),
                pre: vec![Fact::new(2, 1)],
                eff: vec![
                    Effect::new(Fact::new(0, 1)),
                    Effect {
                        conditions: vec![Fact::new(0, 0)],
                        fact: Fact::new(1, 2),
                    },
                ],
                cost: 1,
            }],
            initial: State(vec![0, 0, 1]),
            goal: vec![Fact::new(1, 2)],
            axioms: vec![],
        };
        let config = Config {
            partitioning: Partitioning::Conjunctive {
                cluster_node_limit: 0,
            },
            ..tiny_config()
        };
        let model = Model::<Oxidd>::new(&task, &config).unwrap();

        assert_eq!(model.concrete.groups[0].cluster_count(), 3);
        assert!(has_cross_condition(&task.operators[0]));
        for states in probe_sets(&model) {
            for direction in [Direction::Forward, Direction::Backward] {
                let union = operator_union(&model, &states, direction);
                assert!(model.successors(&states, direction).unwrap() == union);
            }
        }

        assert_eq!(model.bfs(Direction::Forward).unwrap(), Some(vec![0]));
    }

    /// With small node limits, `SymBA*` installs informative (not blind)
    /// abstraction heuristics and still finds a plan.
    #[test]
    fn symba_installs_informative_abstractions() {
        for limit in [1, 4, 8, 16] {
            let mut installed = 0;
            for seed in 0..16 {
                let task = random(seed);
                let model = Model::<Oxidd>::new(&task, &tiny_config()).unwrap();

                let config = SymbaConfig {
                    frontier_node_limit: limit,
                    abstract_node_limit: limit,
                    abstract_slice: usize::MAX,
                    ..SymbaConfig::default()
                };
                let (plan, stats) = model.symba(&config).unwrap();

                installed += stats.installed;
                if let Some(plan) = plan {
                    assert!(task.validate_plan(&plan).is_some());
                }
            }

            assert!(installed > 0, "limit {limit}");
        }
    }
}
