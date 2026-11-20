//! A state space to search: relation groups plus the initial and goal sets.
//!
//! The concrete space comes from the task; projected spaces come from
//! domain abstractions (see `model::abstraction`). Searches only see this
//! type, so the same code runs on both.

use crate::backend::Backend;
use crate::model::Direction;
use crate::relation::Relation;

/// Relation groups sorted by cost, and the two ends of the search.
pub(crate) struct Space<B: Backend> {
    /// Relation groups, each of one cost, sorted by cost: merged
    /// disjunctive groups or conjunctively clustered operators. Projected
    /// spaces hold monolithic groups.
    pub groups: Vec<Relation<B>>,
    /// The initial states (one state in the concrete space).
    pub initial: B::Set,
    /// The goal states, restricted to valid codes.
    pub goal: B::Set,
}

impl<B: Backend> Space<B> {
    /// Where a search in `direction` starts: the initial states forward, the
    /// goal states backward. The start of the reverse direction is the
    /// target.
    pub(crate) fn start(&self, direction: Direction) -> &B::Set {
        match direction {
            Direction::Forward => &self.initial,
            Direction::Backward => &self.goal,
        }
    }
}
