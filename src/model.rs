//! The symbolic model of a task and the layered breadth-first search on it.
//!
//! ```mermaid
//! graph LR
//!     Task --> Encoding
//!     Task --> Relations[per-operator relations]
//!     Encoding --> Relations
//!     Relations --> Explore[explore: layers until target hit]
//!     Explore --> Trace[trace: walk layers back to a plan]
//! ```

use sas_plus::{OpId, Task};

use crate::Error;
use crate::backend::{Backend, BackendResult, Capacity};
use crate::encoding::Encoding;
use crate::relation::OperatorRelation;

/// Search direction of one symbolic step.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Direction {
    /// Successors via [`OperatorRelation::image`].
    Forward,
    /// Predecessors via [`OperatorRelation::preimage`].
    Backward,
}

impl Direction {
    /// The other direction; plan extraction walks against the search.
    fn reverse(self) -> Self {
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

/// A SAS+ task compiled to decision diagrams on backend `B`.
pub(crate) struct Model<B: Backend> {
    /// The diagram manager that owns every set below.
    backend: B,
    /// Bit layout of the task's variables.
    encoding: Encoding,
    /// One relation per operator, in operator id order.
    relations: Vec<OperatorRelation<B>>,
    /// The singleton set of the initial state.
    initial: B::Set,
    /// The goal states, restricted to valid codes.
    goal: B::Set,
}

impl<B: Backend> Model<B> {
    /// Compiles `task` into diagrams with the given capacity.
    ///
    /// # Errors
    ///
    /// - [`Error::InvalidTask`] when [`Task::validate`] fails.
    /// - [`Error::Axioms`] when the task has derived variables.
    /// - [`Error::TooManyBits`] when the encoding exceeds `u32::MAX` diagram variables.
    /// - [`Error::OutOfMemory`] when the diagrams do not fit into `capacity`.
    pub(crate) fn new(task: &Task, capacity: Capacity) -> Result<Self, Error> {
        task.validate().map_err(Error::InvalidTask)?;
        if !task.axioms.is_empty() || task.variables.iter().any(sas_plus::Variable::is_derived) {
            return Err(Error::Axioms);
        }

        let encoding = Encoding::new(task).ok_or(Error::TooManyBits)?;
        let backend = B::new(encoding.num_diagram_vars(), capacity)?;

        let relations = task
            .operators
            .iter()
            .enumerate()
            .map(|(op, operator)| OperatorRelation::new(&backend, &encoding, op, operator))
            .collect::<BackendResult<Vec<_>>>()?;
        let initial = encoding.state(&backend, &task.initial)?;
        let goal = backend.and(
            &encoding.facts(&backend, &task.goal)?,
            &encoding.all_valid(&backend)?,
        )?;

        Ok(Self {
            backend,
            encoding,
            relations,
            initial,
            goal,
        })
    }

    /// Unit-cost breadth-first search in `direction`, then plan extraction.
    ///
    /// Returns a shortest plan, or `None` when the goal is unreachable.
    ///
    /// # Errors
    ///
    /// [`Error::OutOfMemory`] when the node table fills up.
    pub(crate) fn bfs(&self, direction: Direction) -> Result<Option<Vec<OpId>>, Error> {
        let (start, target) = match direction {
            Direction::Forward => (&self.initial, &self.goal),
            Direction::Backward => (&self.goal, &self.initial),
        };
        let Some(found) = self.explore(start, target, direction)? else {
            return Ok(None);
        };

        let mut plan = self.trace(&found, direction.reverse())?;
        if direction == Direction::Forward {
            plan.reverse();
        }

        Ok(Some(plan))
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

    /// One step from `states` in `direction`, over all operators.
    ///
    /// # Errors
    ///
    /// Out of memory in the backend.
    fn successors(&self, states: &B::Set, direction: Direction) -> BackendResult<B::Set> {
        self.relations
            .iter()
            .try_fold(self.backend.constant(false), |acc, relation| {
                self.backend
                    .or(&acc, &self.step(relation, states, direction)?)
            })
    }

    /// One step of `relation` from `states` in `direction`.
    ///
    /// # Errors
    ///
    /// Out of memory in the backend.
    fn step(
        &self,
        relation: &OperatorRelation<B>,
        states: &B::Set,
        direction: Direction,
    ) -> BackendResult<B::Set> {
        match direction {
            Direction::Forward => relation.image(&self.backend, states),
            Direction::Backward => relation.preimage(&self.backend, states),
        }
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
        for relation in &self.relations {
            let reached = self
                .backend
                .and(&self.step(relation, state, walk)?, layer)?;
            if !self.backend.is_empty(&reached) {
                return Ok((relation.op, self.pick_state(&reached)?));
            }
        }

        panic!("a BFS layer state has no neighbour in the previous layer");
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
    fn pick_state(&self, states: &B::Set) -> BackendResult<B::Set> {
        let assignment = self
            .backend
            .pick(states)
            .expect("picking from a non-empty set");

        self.encoding
            .state(&self.backend, &self.encoding.decode(&assignment))
    }
}

#[cfg(test)]
mod tests {
    use sas_plus::{Effect, Fact, Operator, State, Variable};

    use super::*;
    use crate::backend::bcdd::Oxidd;

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
        let capacity = Capacity {
            nodes: 1 << 10,
            cache: 1 << 8,
        };
        let model = Model::<Oxidd>::new(&task, capacity).unwrap();
        let b = &model.backend;

        let found = model
            .explore(&model.goal, &model.initial, Direction::Backward)
            .unwrap()
            .expect("the initial state reaches the goal");
        let invalid = b.not(&model.encoding.all_valid(b).unwrap()).unwrap();

        for layer in &found.layers {
            assert!(b.is_empty(&b.and(layer, &invalid).unwrap()));
        }
    }
}
