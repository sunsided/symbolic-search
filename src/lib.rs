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
//!   and next bits and a Gamer-style static variable order
//!   ([`VariableOrder`]).
//! - Per-operator transition relations, including conditional effects,
//!   merged per cost into disjunctive groups up to a node limit
//!   ([`Config::merge_node_limit`]), or split into per-variable conjunctive
//!   clusters with an early-quantification schedule
//!   ([`Partitioning::Conjunctive`]).
//! - Symbolic forward and backward breadth-first search (unit cost).
//! - Symbolic Dijkstra forward, backward and bidirectional (action costs,
//!   zero costs included).
//! - `BDDA*` and symbolic greedy best-first search with a [`Heuristic`]
//!   stored as one BDD per `h` value: symbolic pattern databases
//!   ([`SymbolicTask::pdb`]), canonical PDB collections
//!   ([`SymbolicTask::canonical_pdbs`]) or explicit estimates
//!   ([`SymbolicTask::heuristic`]).
//! - `SymBA*`: bidirectional `A*` with perimeter abstractions that drop
//!   variables or merge values by dropping single bits
//!   ([`AbstractionHierarchy`]), resumable abstract searches, separate concrete and abstract
//!   node limits and an expansion and time budget ([`SymbaConfig`]).
//!
//! Every search reconstructs a plan as operator ids in execution order and
//! returns it with its cost ([`Solution`]). `SymBA*` budgets report the
//! best plan so far and a lower bound ([`Error::BudgetExhausted`]).
//! [`Config::for_task`] sizes the node table from the task, and running out
//! of nodes retries with a larger table up to
//! [`Config::max_node_capacity`]. Conjunctive clusters use a greedy or the
//! IWLS95 quantification schedule ([`Schedule`]). The BDD library sits
//! behind the public [`backend::Backend`] trait.
//!
//! ## Planned
//!
//! Axioms, mutex pruning, full merge-and-shrink heuristics, time-based step
//! estimates for `SymBA*`. See `docs/design.md` in the
//! repository.
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
//!     ..Config::default()
//! };
//! let symbolic = SymbolicTask::new(&task, config)?;
//! let solution = symbolic.forward_bfs()?.expect("the goal is reachable");
//! assert_eq!(solution.plan, [0, 1]);
//! assert_eq!(solution.cost, 2);
//! # Ok::<(), symbolic_search::Error>(())
//! ```
//!
//! ## Status
//!
//! Pre-release (0.0.0). The API will change.
//!
//! Repository: <https://github.com/sunsided/symbolic-search>

pub mod backend;
mod encoding;
mod error;
mod model;
mod order;
mod relation;
mod search;

pub use error::Error;
pub use order::VariableOrder;
pub use search::{
    AbstractionHierarchy, Config, Heuristic, Partitioning, Schedule, Solution, SymbaConfig,
    SymbolicTask,
};
