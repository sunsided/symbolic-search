//! The public entry point: a task compiled to BDDs, and its searches.

use sas_plus::{OpId, Task};

use crate::Error;
use crate::backend::Capacity;
use crate::backend::bcdd::Oxidd;
use crate::model::{Direction, Model};

/// Resource limits of a [`SymbolicTask`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Config {
    /// Maximum number of live BDD nodes. The node table is reserved at this
    /// size when the task is compiled and never grows; searches that need
    /// more nodes fail with [`Error::OutOfMemory`].
    pub node_capacity: usize,
    /// Number of entries of the operation cache. Larger caches avoid
    /// recomputation at the price of memory.
    pub cache_capacity: usize,
}

impl Default for Config {
    /// 2²⁰ nodes and 2¹⁸ cache entries.
    fn default() -> Self {
        Self {
            node_capacity: 1 << 20,
            cache_capacity: 1 << 18,
        }
    }
}

/// A SAS+ task compiled to binary decision diagrams.
///
/// Holds the encoding, one transition relation per operator, and the
/// initial and goal sets. Searches reuse them and can be run repeatedly.
pub struct SymbolicTask {
    /// The compiled model on the default backend.
    model: Model<Oxidd>,
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
        let capacity = Capacity {
            nodes: config.node_capacity,
            cache: config.cache_capacity,
        };

        Ok(Self {
            model: Model::new(task, capacity)?,
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
}
