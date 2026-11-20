//! # Symbolic search
//!
//! Symbolic state-space search over SAS+ tasks with binary decision diagrams
//! (BDDs). Sets of states are BDDs over a binary encoding of the task's
//! variables; one search step maps a whole layer of states at once through
//! the operators' transition relations.
//!
//! ## Implemented
//!
//! - Binary encoding of [`sas_plus::Task`] states, with interleaved current
//!   and next bits.
//! - Per-operator transition relations, including conditional effects.
//! - Symbolic forward and backward breadth-first search (unit cost) with
//!   layered plan reconstruction.
//!
//! ## Planned
//!
//! Symbolic Dijkstra, bidirectional search, `BDDA*`, `SymBA*` and symbolic
//! abstraction heuristics. See `docs/design.md` in the repository.
//!
//! ## Example
//!
//! ```
//! use sas_plus::{Effect, Fact, Operator, State, Task, Variable};
//! use symbolic_search::{Config, SymbolicTask};
//!
//! // One counter from 0 to 2, incremented by two operators.
//! let step = |from, to| Operator {
//!     name: format!("inc{from}"),
//!     pre: vec![Fact::new(0, from)],
//!     eff: vec![Effect::new(Fact::new(0, to))],
//!     cost: 1,
//! };
//! let task = Task {
//!     variables: vec![Variable::new("counter", 3)],
//!     operators: vec![step(0, 1), step(1, 2)],
//!     initial: State(vec![0]),
//!     goal: vec![Fact::new(0, 2)],
//!     axioms: vec![],
//! };
//!
//! let config = Config {
//!     node_capacity: 1 << 10,
//!     cache_capacity: 1 << 8,
//! };
//! let symbolic = SymbolicTask::new(&task, config)?;
//! let plan = symbolic.forward_bfs()?.expect("the goal is reachable");
//! assert_eq!(plan, [0, 1]);
//! # Ok::<(), symbolic_search::Error>(())
//! ```
//!
//! ## Status
//!
//! Pre-release (0.0.0). The API will change.
//!
//! Repository: <https://github.com/sunsided/symbolic-search>

mod backend;
mod encoding;
mod error;
mod model;
mod relation;
mod search;

pub use error::Error;
pub use search::{Config, SymbolicTask};
