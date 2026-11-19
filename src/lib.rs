//! # Symbolic search
//!
//! Symbolic state-space search with decision diagrams: symbolic BFS, bidirectional search, `BDDA*` and `SymBA*`.
//!
//! ## Planned scope
//!
//! - Symbolic BFS and Dijkstra
//! - Symbolic bidirectional search
//! - `BDDA*`
//! - `SymBA*`
//! - Transition relations, image and preimage, variable ordering
//!
//! ## Status
//!
//! Pre-release. Version 0.0.0 reserves the name; the API is not started yet and will change.
//!
//! Repository: <https://github.com/sunsided/symbolic-search>

#![no_std]

/// Transitions over sets of states, as used by symbolic search.
pub trait SymbolicTransitions {
    /// A set of states, typically a decision diagram.
    type Set;

    /// The states reachable from `states` by one transition.
    fn image(&self, states: &Self::Set) -> Self::Set;

    /// The states that reach `states` by one transition.
    fn preimage(&self, states: &Self::Set) -> Self::Set;
}
