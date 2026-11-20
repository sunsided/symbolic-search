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
    one BDD per `h` value, from symbolic pattern databases or explicit
    estimates;
  - SymBA\*: bidirectional A\* with perimeter abstractions that drop
    variables, resumable abstract searches per direction, separate concrete
    and abstract node limits, and an expansion and time budget
    (`SymbaConfig`).

  All searches reconstruct plans. BDDs come from
  [`oxidd`](https://crates.io/crates/oxidd) behind an internal backend trait.
- Not done: axioms (rejected with `Error::Axioms`), mutex pruning, merge-and-shrink heuristics.

See [`docs/design.md`](docs/design.md) for the design and milestones.

## License

Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE))
- MIT license ([LICENSE-MIT](LICENSE-MIT))

at your option.
