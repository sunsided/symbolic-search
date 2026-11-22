# symbolic-search

[![crates.io](https://img.shields.io/crates/v/symbolic-search.svg)](https://crates.io/crates/symbolic-search)
[![docs.rs](https://img.shields.io/docsrs/symbolic-search)](https://docs.rs/symbolic-search)
[![CI](https://github.com/sunsided/symbolic-search/actions/workflows/ci.yml/badge.svg)](https://github.com/sunsided/symbolic-search/actions/workflows/ci.yml)
[![codecov](https://codecov.io/gh/sunsided/symbolic-search/graph/badge.svg)](https://codecov.io/gh/sunsided/symbolic-search)
[![MSRV](https://img.shields.io/crates/msrv/symbolic-search)](https://crates.io/crates/symbolic-search)
[![License](https://img.shields.io/crates/l/symbolic-search)](https://github.com/sunsided/symbolic-search#license)
[![unsafe forbidden](https://img.shields.io/badge/unsafe-forbidden-success.svg)](https://github.com/rust-secure-code/safety-dance/)

Symbolic state-space search for SAS+ planning tasks from [sas-plus](https://github.com/sunsided/sas-plus), with
binary decision diagrams (BDDs) as the state-set representation. Instead of expanding one state at a time, every
search step maps a whole layer of states through the operators' transition relations at once, which is what makes
symbolic search strong on cost-optimal planning tasks with large, regular state spaces. The crate implements
symbolic BFS and Dijkstra (forward, backward and bidirectional), BDDA\* and greedy best-first search with symbolic
abstraction heuristics, and SymBA\* (the bidirectional A\* with perimeter abstractions of Torralba et al.), and every search
returns a reconstructed plan with its cost.

BDDs come from [oxidd](https://crates.io/crates/oxidd) behind a sealed backend trait; a small plain-Rust reference
BDD implements the same trait and is used to cross-check oxidd in the tests. Node tables are sized from the task and
grow on demand, so a search that runs out of nodes retries with a larger table instead of failing.

Part of the [planning and search crates](https://sunsided.github.io/planning-and-search/); see also this crate's [overview page](https://sunsided.github.io/planning-and-search/crates/symbolic-search/).

## Who this is for

- Planning researchers working on cost-optimal planning who want BDD-based search (SymBA\*, BDDA\*, symbolic PDBs,
  symbolic merge-and-shrink) on the same task representation as the other planning crates, without wrapping
  Gamer or Fast Downward's symbolic search.
- Planner authors who need optimal plans, or a proof that no plan exists, on tasks whose state spaces are too large
  for explicit search but regular enough for compact BDDs.
- Anyone experimenting with transition-relation partitioning (disjunctive groups, conjunctive clusters with greedy
  or IWLS95 schedules) or variable orders, with a reference BDD to check results against.
- For explicit-state heuristic search, use [heuristic-search](https://github.com/sunsided/heuristic-search) with
  heuristics from [planning-heuristics](https://github.com/sunsided/planning-heuristics); for tasks that factor
  into a center and loosely coupled leaves, [decoupled-search](https://github.com/sunsided/decoupled-search) is
  often the better fit.

## Installation

```toml
[dependencies]
symbolic-search = "0.1"
```

The [crate documentation](https://docs.rs/symbolic-search) has a complete example.

## Scope

- Binary encoding of `sas_plus::Task` states with a Gamer-style static variable order.
- Per-operator transition relations (with conditional effects), merged per cost into disjunctive groups up to a
  node limit, or split into per-variable conjunctive clusters with early quantification (greedy or IWLS95
  schedule).
- Symbolic forward and backward BFS (unit cost).
- Symbolic Dijkstra forward, backward and bidirectional (action costs, zero costs included).
- BDDA\* and symbolic greedy best-first search with heuristics stored as one BDD per `h` value, from symbolic
  pattern databases, canonical PDB collections (maximum over additive subsets, capped), symbolic
  merge-and-shrink (linear merging, h-preserving node-limited shrinking) or explicit estimates.
- SymBA\*: bidirectional A\* with perimeter abstractions that drop variables or single bits (value merging), or
  merge-and-shrink perimeters, resumable abstract searches per direction, separate concrete and abstract node
  limits, and an expansion and time budget (`SymbaConfig`). An exhausted budget reports the best plan found so far
  and a lower bound.
- `Config::for_task` sizes the node table from the task; running out of nodes retries with a larger table up to
  `Config::max_node_capacity`.
- The BDD library sits behind the sealed `backend::Backend` trait; `SymbolicTask::with_backend` also runs on
  `backend::ReferenceBdd`.

Not yet implemented: axioms (rejected with `Error::Axioms`), mutex pruning, non-linear merge strategies, exact
bisimulation and label reduction. The design is described in
[docs/design.md](https://github.com/sunsided/symbolic-search/blob/main/docs/design.md).

## Minimum supported Rust version

Rust 1.91 (edition 2024). The MSRV applies to the library and its tests. It may be raised
in a minor release when a dependency or a language feature calls for it.

## Contributing

See [CONTRIBUTING.md](https://github.com/sunsided/symbolic-search/blob/main/CONTRIBUTING.md).

## License

Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](https://github.com/sunsided/symbolic-search/blob/main/LICENSE-APACHE))
- MIT license ([LICENSE-MIT](https://github.com/sunsided/symbolic-search/blob/main/LICENSE-MIT))

at your option.

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion
in this work by you, as defined in the Apache-2.0 license, shall be dual licensed as above,
without any additional terms or conditions.
