//! Integration tests: symbolic BFS against an explicit-state BFS oracle.

mod fixtures;
mod handmade;
mod oracle;
mod random;

use sas_plus::Task;
use symbolic_search::{Config, SymbolicTask};

/// Small BDD limits, so tests stay light on memory.
const SMALL: Config = Config {
    node_capacity: 1 << 15,
    cache_capacity: 1 << 12,
};

/// Plans of both symbolic searches on `task`, forward first.
fn symbolic_plans(task: &Task) -> (Option<Vec<usize>>, Option<Vec<usize>>) {
    let symbolic = SymbolicTask::new(task, SMALL).expect("task compiles");
    let forward = symbolic.forward_bfs().expect("forward search fits");
    let backward = symbolic.backward_bfs().expect("backward search fits");

    (forward, backward)
}

/// Checks both symbolic searches on `task` against the oracle and returns the
/// optimal unit-cost plan length (`None` when unsolvable).
fn check_against_oracle(task: &Task) -> Option<usize> {
    let expected = oracle::shortest_plan_length(task);
    let (forward, backward) = symbolic_plans(task);

    for plan in [&forward, &backward] {
        assert_eq!(
            plan.as_ref().map(Vec::len),
            expected,
            "plan length is optimal"
        );
        if let Some(plan) = plan {
            assert!(task.validate_plan(plan).is_some(), "plan {plan:?} is valid");
        }
    }

    expected
}
