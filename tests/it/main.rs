//! Integration tests: symbolic searches against explicit-state BFS and
//! Dijkstra oracles, with every plan checked by `Task::validate_plan`.

mod backends;
mod capacity;
mod costs;
mod fixtures;
mod handmade;
mod heuristics;
mod oracle;
mod properties;
mod random;

use sas_plus::Task;
use symbolic_search::{Config, Partitioning, Schedule, Solution, SymbolicTask, VariableOrder};

/// Small BDD limits, so tests stay light on memory. Merged relation groups
/// and the Gamer order.
const SMALL: Config = Config {
    node_capacity: 1 << 15,
    cache_capacity: 1 << 12,
    max_node_capacity: 1 << 15,
    merge_node_limit: 1 << 10,
    variable_order: VariableOrder::Gamer,
    partitioning: Partitioning::Disjunctive,
};

/// [`SMALL`] with one relation per operator and the task's variable order,
/// the other extreme of both options.
const PER_OPERATOR: Config = Config {
    merge_node_limit: 0,
    variable_order: VariableOrder::Task,
    ..SMALL
};

/// [`SMALL`] with conjunctive partitioning, one cluster per conjunct, so
/// early quantification runs at its finest, in the IWLS95 order.
const CONJUNCTIVE: Config = Config {
    partitioning: Partitioning::Conjunctive {
        cluster_node_limit: 0,
        schedule: Schedule::Iwls95,
    },
    ..SMALL
};

/// Both option extremes of the disjunctive partitioning, and the
/// conjunctive one; every oracle check runs on each.
const CONFIGS: [Config; 3] = [SMALL, PER_OPERATOR, CONJUNCTIVE];

/// Plans of both symbolic BFS directions on `task` under `config`, forward
/// first.
fn symbolic_plans_with(task: &Task, config: Config) -> (Option<Vec<usize>>, Option<Vec<usize>>) {
    let symbolic = SymbolicTask::new(task, config).expect("task compiles");
    let forward = symbolic.forward_bfs().expect("forward search fits");
    let backward = symbolic.backward_bfs().expect("backward search fits");
    let plan = |solution: Option<Solution>| {
        solution.map(|s| {
            assert_eq!(validated_cost(task, &s, "bfs"), s.cost);
            s.plan
        })
    };

    (plan(forward), plan(backward))
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

/// The cost of `solution`'s plan by the independent validator, panicking
/// with `what` when the plan is invalid or the reported cost is wrong.
fn validated_cost(task: &Task, solution: &Solution, what: &str) -> u64 {
    let plan = &solution.plan;
    let cost = task
        .validate_plan(plan)
        .unwrap_or_else(|| panic!("{what}: plan {plan:?} is invalid"));
    assert_eq!(cost, solution.cost, "{what}: reported cost");

    cost
}
