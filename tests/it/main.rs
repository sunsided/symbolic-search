//! Integration tests: symbolic searches against explicit-state BFS and
//! Dijkstra oracles, with every plan checked by `Task::validate_plan`.

mod costs;
mod fixtures;
mod handmade;
mod heuristics;
mod oracle;
mod random;

use sas_plus::Task;
use symbolic_search::{Config, SymbolicTask, VariableOrder};

/// Small BDD limits, so tests stay light on memory. Merged relation groups
/// and the Gamer order.
const SMALL: Config = Config {
    node_capacity: 1 << 15,
    cache_capacity: 1 << 12,
    merge_node_limit: 1 << 10,
    variable_order: VariableOrder::Gamer,
};

/// [`SMALL`] with one relation per operator and the task's variable order,
/// the other extreme of both options.
const PER_OPERATOR: Config = Config {
    merge_node_limit: 0,
    variable_order: VariableOrder::Task,
    ..SMALL
};

/// Both option extremes; every oracle check runs on each.
const CONFIGS: [Config; 2] = [SMALL, PER_OPERATOR];

/// Plans of both symbolic BFS directions on `task` under `config`, forward
/// first.
fn symbolic_plans_with(task: &Task, config: Config) -> (Option<Vec<usize>>, Option<Vec<usize>>) {
    let symbolic = SymbolicTask::new(task, config).expect("task compiles");
    let forward = symbolic.forward_bfs().expect("forward search fits");
    let backward = symbolic.backward_bfs().expect("backward search fits");

    (forward, backward)
}

/// Plans of both symbolic BFS directions on `task` under [`SMALL`].
fn symbolic_plans(task: &Task) -> (Option<Vec<usize>>, Option<Vec<usize>>) {
    symbolic_plans_with(task, SMALL)
}

/// Checks both symbolic BFS directions on `task`, under every config in
/// [`CONFIGS`], against the oracle and returns the optimal unit-cost plan
/// length (`None` when unsolvable).
fn check_against_oracle(task: &Task) -> Option<usize> {
    let expected = oracle::shortest_plan_length(task);
    for config in CONFIGS {
        let (forward, backward) = symbolic_plans_with(task, config);
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
    }

    expected
}

/// The cost of `plan` by the independent validator, panicking with `what`
/// when the plan is invalid.
fn validated_cost(task: &Task, plan: &[usize], what: &str) -> u64 {
    task.validate_plan(plan)
        .unwrap_or_else(|| panic!("{what}: plan {plan:?} is invalid"))
}
