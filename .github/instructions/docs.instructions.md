---
applyTo: "**/*.md,docs/**"
---

# Documentation

- README files are rendered on crates.io and docs.rs. Every link to a repo file and every
  image uses an absolute URL (`https://github.com/...`, `https://raw.githubusercontent.com/...`).
  Only in-page anchors may be relative.
- Mermaid diagrams (```` ```mermaid ````) must be valid (`task lint:mermaid` renders them)
  and must match the code. Flag diagrams that the change makes stale. Keep each diagram under
  about 15 nodes.
- Code blocks are tagged with their language. `README.md` has no Rust examples; the
  crate-level docs in `src/lib.rs` hold the doctested example. If a README gains a `rust`
  block, include it in a private `#[cfg(doctest)]` module in `src/lib.rs` in the same change.
- `docs/design.md` describes the pipeline, the module map and the milestones. Keep its
  mermaid pipeline, code map and milestone list in step with `src/`.
- Keep badges, the License section and the MSRV statement consistent with `Cargo.toml`.
