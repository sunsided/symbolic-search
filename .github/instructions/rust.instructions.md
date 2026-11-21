---
applyTo: "**/*.rs"
---

# Rust code

- Every module (`//!`), type, field, variant, function, constant and macro has a doc
  comment, private and `pub(crate)` items included. Docs state intent, invariants, units and
  error/panic conditions (`# Errors`, `# Panics`, `# Safety`). They do not restate the name.
- No `mod.rs`: modules with children are `foo.rs` plus `foo/`. Integration tests live under
  `tests/it/`.
- No separator comments (`// -----`, `// =====`). If a file needs sections, split it into
  submodules.
- Leave a blank line after the closing `}` of a block statement when another statement
  follows, and between logical phases of a function. No lumped code.
- One level of abstraction per function. Long loops (simulations, game ticks, solvers) are
  short drivers that call documented step functions. If a block needs a "what" comment,
  extract it into a function named after that comment.
- `clippy::pedantic` clean. Suppression only with a scoped
  `#[expect(lint, reason = "...")]`. Never `#[allow]`, and never crate-level suppression.
- No `unsafe` unless the crate opts out of `#![forbid(unsafe_code)]` with a documented
  reason. Each unsafe block has one operation and a `// SAFETY:` comment.
- `oxidd` types stay inside `backend` (`backend/bcdd.rs`); the rest of the crate talks to
  the sealed `backend::Backend` trait. Flag any `oxidd` use outside `backend`.
- Every search returns a `Solution` whose plan passes `Task::validate_plan` with exactly the
  reported cost. Cost-optimal searches (Dijkstra, `SymBA*`, `BDDA*` with an admissible
  heuristic) must match the explicit oracles in `tests/it/oracle.rs`.
- Heuristics (PDBs, canonical PDBs, merge-and-shrink) are admissible and report a dead end
  only for real dead ends. `backend::ReferenceBdd` must give the same results as `Oxidd`.
- Running out of nodes returns `Error::OutOfMemory` (after the capacity retries), never a
  panic. Invalid tasks, patterns, facts and states return an `Error`, never a panic; the
  `task_input` fuzz target checks that.
- New search features get oracle checks in `tests/it/`, and a property in
  `tests/it/properties.rs` or a fuzz target when they take new input or cost arithmetic.
- Public API changes go with a matching update to the crate docs in `src/lib.rs`, the
  README scope list and `docs/design.md`.
- Mermaid diagrams in doc comments must match the code they describe. Update them when the
  structure changes.
