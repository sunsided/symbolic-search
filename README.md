# symbolic-search

Symbolic state-space search with decision diagrams: symbolic BFS, bidirectional search, BDDA* and SymBA*.

## Status

Pre-release (0.0.0); the API will change.

- Done:
  - binary encoding of `sas_plus::Task` states with a Gamer-style static
    variable order;
  - per-operator transition relations (with conditional effects), merged per
    cost into disjunctive groups up to a node limit, or split into
    per-variable conjunctive clusters with early quantification;
  - symbolic forward and backward BFS (unit cost);
  - symbolic Dijkstra forward, backward and bidirectional (action costs,
    zero costs included);
  - BDDA\* and symbolic greedy best-first search with heuristics stored as
    one BDD per `h` value, from symbolic pattern databases, canonical PDB
    collections (maximum over additive subsets, capped), symbolic
    merge-and-shrink (linear merging, h-preserving node-limited shrinking)
    or explicit estimates;
  - SymBA\*: bidirectional A\* with perimeter abstractions that drop
    variables or single bits (value merging), or merge-and-shrink
    perimeters, resumable abstract searches per direction, separate concrete
    and abstract node limits, and an expansion and time budget
    (`SymbaConfig`).

  All searches return a `Solution` (plan and cost). SymBA\* budgets
  report the best plan found so far and a lower bound.
  `Config::for_task` sizes the node table from the task, and running out
  of nodes retries with a larger table up to `Config::max_node_capacity`.
  Conjunctive clusters use a greedy or the IWLS95 schedule.
  All searches reconstruct plans. BDDs come from
  [`oxidd`](https://crates.io/crates/oxidd) behind a public backend trait
  (`backend::Backend`, sealed); `SymbolicTask::with_backend` also runs on
  `backend::ReferenceBdd`, a plain-Rust BDD that cross-checks oxidd.
- Not done: axioms (rejected with `Error::Axioms`), mutex pruning,
  non-linear merge strategies, exact bisimulation and label reduction.

See [`docs/design.md`](docs/design.md) for the design and milestones.

## License

Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE))
- MIT license ([LICENSE-MIT](LICENSE-MIT))

at your option.
