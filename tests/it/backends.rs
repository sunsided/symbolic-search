//! The plain-Rust reference backend against oxidd and the oracles.

use sas_plus::Task;
use sas_plus::random::{RandomTaskParams, random_task};
use symbolic_search::backend::{Backend, ReferenceBdd};
use symbolic_search::{Config, Error, Solution, SymbolicTask};

use crate::{SMALL, oracle, validated_cost};

/// The reference backend never frees nodes, so it gets a larger table.
const REFERENCE: Config = Config {
    node_capacity: 1 << 18,
    max_node_capacity: 1 << 20,
    ..SMALL
};

/// Costs of every search on `B`, in a fixed order, checked by the
/// validator.
fn costs<B: Backend>(task: &Task, config: Config) -> Vec<Option<u64>> {
    let symbolic = SymbolicTask::<B>::with_backend(task, config).expect("task compiles");
    let pdb = symbolic.pdb(&[0]).expect("pdb fits");
    let runs: [Result<Option<Solution>, Error>; 6] = [
        symbolic.forward_bfs().map(|s| {
            s.map(|s| Solution {
                cost: s.plan.len() as u64,
                ..s
            })
        }),
        symbolic.forward_dijkstra(),
        symbolic.backward_dijkstra(),
        symbolic.bidirectional_dijkstra(),
        symbolic.symba(1),
        symbolic.astar(&pdb),
    ];

    runs.into_iter()
        .enumerate()
        .map(|(i, run)| {
            run.expect("search fits").map(|s| {
                if i > 0 {
                    validated_cost(task, &s, "reference cross-check");
                }
                s.cost
            })
        })
        .collect()
}

/// On random tasks with conditional effects, both backends give the same
/// plan lengths and costs, which match the oracles.
#[test]
fn reference_backend_matches_oxidd_and_oracles() {
    let params = RandomTaskParams {
        max_domain: 5,
        max_conditions: 1,
        ..RandomTaskParams::default()
    };
    for seed in 0..24 {
        let task = random_task(seed, params);
        let reference = costs::<ReferenceBdd>(&task, REFERENCE);
        let oxidd = costs::<symbolic_search::backend::Oxidd>(&task, SMALL);

        assert_eq!(reference, oxidd, "seed {seed}");
        let length = oracle::shortest_plan_length(&task).map(|n| n as u64);
        assert_eq!(reference[0], length, "seed {seed}");
        for cost in &reference[1..] {
            assert_eq!(*cost, oracle::cheapest_plan_cost(&task), "seed {seed}");
        }
    }
}
