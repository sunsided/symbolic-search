# symbolic-search

Symbolic state-space search with decision diagrams: symbolic BFS, bidirectional search, BDDA* and SymBA*.

## Status

Pre-release (0.0.0); the API will change.

- Done: binary encoding of `sas_plus::Task` states, per-operator transition
  relations (with conditional effects), symbolic forward and backward BFS
  (unit cost) with plan reconstruction. BDDs come from
  [`oxidd`](https://crates.io/crates/oxidd) behind an internal backend trait.
- Planned: symbolic Dijkstra, bidirectional search, BDDA*, SymBA*, symbolic
  abstraction heuristics. Axioms are not supported yet.

See [`docs/design.md`](docs/design.md) for the design and milestones.

## License

Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE))
- MIT license ([LICENSE-MIT](LICENSE-MIT))

at your option.
