# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.1.0](https://github.com/sunsided/symbolic-search/releases/tag/v0.1.0) - 2026-11-21

### Added

- Add CONTRIBUTING.md
- Add README badges and absolute links, and trim the package
- Add cargo-fuzz targets for task input and search costs
- Add property tests over random task parameters
- Add REUSE licensing metadata
- Add cargo-deny, Dependabot and typos
- Add coverage reporting with Codecov
- Add the Taskfile and the CI workflows
- Add symbolic merge-and-shrink, heuristic retries and a subset cap
- Add a plain-Rust reference BDD backend and seal the trait
- Add the IWLS95 schedule, bit abstractions and canonical PDB collections
- Add conjunctive partitioning with early quantification
- Add cost-aware symbolic search: Dijkstra, BDDA*, GBFS, SymBA*, PDBs
- Add symbolic forward and backward BFS on oxidd BDDs
- Add the design document for symbolic search

### Changed

- Prepare the 0.1.0 release
- Describe the released scope and audience in the README
- Stop tracking Cargo.lock
- Extract helpers from two functions over the line limit
- Adopt the strict lint table and forbid unsafe code
- Render mermaid diagrams in rustdoc and on docs.rs
- Track Cargo.lock
- Tune SymbaConfig on larger random tasks and benchmark one
- Hide oxidd types, count abstract work, and widen the tests
- Tune the SymbaConfig defaults on the fixtures
- Make the backend trait public and the task generic over it
- Derive capacities from the task and retry on running out of nodes
- Return plans with their cost, and the best plan on budget exhaustion
- Tighten the partitioning and SymBA* tests and docs
- Resume SymBA* abstract searches and add SymbaConfig
- Make SymBA* perimeters informative at small node limits
- Document the cost-aware searches and update the status
- Benchmark the searches on the gripper-costs fixture
- Tighten design claims and cap the doctest node table
- Test symbolic BFS against an explicit-state oracle
- Reserve the crate name with a 0.0.0 placeholder

### Fixed

- Fix empty PDB collections, retune on cold runs, and widen the checks
