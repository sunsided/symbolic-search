//! Property tests: the seed sweeps of the other modules, generalised over
//! the generator parameters.
//!
//! The sweeps fix `RandomTaskParams` per test; here proptest draws the seed
//! and the parameters together, so variable counts, domain sizes, operator
//! counts, cost ranges and conditional effects vary in one run. Raise the
//! case count with `task test:prop` (`PROPTEST_CASES`).

use proptest::prelude::*;
use sas_plus::Task;
use sas_plus::random::{RandomTaskParams, random_task};

use crate::{check_against_oracle, costs, heuristics};

/// Generator parameters for tasks small enough for the explicit oracles:
/// one to five variables of domain up to 5, at most 12 operators of cost
/// 1 to 5, up to two conditions per effect and no derived variables (the
/// symbolic encoding rejects axioms).
fn params() -> impl Strategy<Value = RandomTaskParams> {
    (1..=5_usize, 2..=5_u32, 0..=12_usize, 1..=5_u32, 0..=2_usize).prop_map(
        |(num_vars, max_domain, num_ops, max_cost, max_conditions)| RandomTaskParams {
            num_vars,
            max_domain,
            num_ops,
            max_cost,
            max_conditions,
            num_derived: 0,
        },
    )
}

/// A random task from an arbitrary seed and [`params`].
fn task() -> impl Strategy<Value = Task> {
    (any::<u64>(), params()).prop_map(|(seed, params)| random_task(seed, params))
}

proptest! {
    /// Both BFS directions find shortest plans under every config.
    #[test]
    fn bfs_matches_oracle(task in task()) {
        check_against_oracle(&task);
    }

    /// Every cost-optimal search (Dijkstra in all directions, `SymBA*`)
    /// finds a cheapest plan under every config.
    #[test]
    fn cost_optimal_searches_match_oracle(task in task()) {
        costs::check_costs(&task);
    }

    /// PDBs, canonical PDB collections and merge-and-shrink are admissible
    /// and only call real dead ends dead, and `BDDA*` with them is optimal.
    #[test]
    fn heuristics_are_admissible_and_astar_optimal(task in task()) {
        heuristics::check_pdb_values(&task);
        heuristics::check_canonical_values(&task);
        heuristics::check_heuristic_search(&task);
    }
}
