//! Cost-optimal searches (Dijkstra in all directions, `SymBA*`) against the
//! explicit-state Dijkstra oracle.

use sas_plus::random::{RandomTaskParams, random_task};
use sas_plus::{Effect, Fact, Operator, State, Task, Variable};
use symbolic_search::{SymbaConfig, SymbolicTask};

use crate::{CONFIGS, SMALL, oracle, validated_cost};

/// `SymBA*` parameters that abstract at once and pause abstract searches
/// after every expansion, so they are resumed many times.
const RESUMING: SymbaConfig = SymbaConfig {
    frontier_node_limit: 1,
    abstract_node_limit: 64,
    abstract_slice: 1,
    max_expansions: None,
    time_limit: None,
};

/// `SymBA*` parameters with slices of 2 and a small abstract limit, so
/// paused searches sit on top of frozen levels.
const FREEZING: SymbaConfig = SymbaConfig {
    abstract_node_limit: 8,
    abstract_slice: 2,
    ..RESUMING
};

/// Checks every cost-optimal search on `task`, under every config, against
/// the oracle and returns the optimal plan cost (`None` when unsolvable).
pub fn check_costs(task: &Task) -> Option<u64> {
    let expected = oracle::cheapest_plan_cost(task);
    for config in CONFIGS {
        let symbolic = SymbolicTask::new(task, config).expect("task compiles");
        let runs = [
            ("forward dijkstra", symbolic.forward_dijkstra()),
            ("backward dijkstra", symbolic.backward_dijkstra()),
            ("bidirectional dijkstra", symbolic.bidirectional_dijkstra()),
            ("symba, abstracting at once", symbolic.symba(1)),
            ("symba, abstracting later", symbolic.symba(8)),
            ("symba, never abstracting", symbolic.symba(usize::MAX)),
            (
                "symba, resuming abstract searches",
                symbolic.symba_with(&RESUMING),
            ),
            (
                "symba, resuming and freezing",
                symbolic.symba_with(&FREEZING),
            ),
        ];
        for (what, plan) in runs {
            let plan = plan.unwrap_or_else(|e| panic!("{what}: {e}"));
            let cost = plan.map(|plan| validated_cost(task, &plan, what));
            assert_eq!(cost, expected, "{what} is optimal");
        }
    }

    expected
}

/// Tally of a cost sweep, to prove it covers every outcome.
#[derive(Debug, Default)]
struct Outcomes {
    /// Solvable with cost 0.
    trivial: usize,
    /// Solvable with a positive cost.
    solved: usize,
    /// Solvable, and the fewest-operator plan of BFS is not the cheapest.
    cost_matters: usize,
    /// No goal state is reachable.
    unsolvable: usize,
}

/// Checks `seeds` tasks from `params` and returns the tally.
fn sweep(params: RandomTaskParams, seeds: u64) -> Outcomes {
    let mut outcomes = Outcomes::default();
    for seed in 0..seeds {
        let task = random_task(seed, params);
        match check_costs(&task) {
            Some(0) => outcomes.trivial += 1,
            Some(optimum) => {
                outcomes.solved += 1;
                let bfs = SymbolicTask::new(&task, SMALL)
                    .and_then(|s| s.forward_bfs())
                    .expect("BFS fits")
                    .expect("solvable");
                if validated_cost(&task, &bfs, "bfs") > optimum {
                    outcomes.cost_matters += 1;
                }
            }
            None => outcomes.unsolvable += 1,
        }
    }

    outcomes
}

/// Asserts that a sweep saw trivial, solved and unsolvable tasks.
fn assert_covers_all(outcomes: &Outcomes) {
    assert!(outcomes.trivial > 0, "{outcomes:?}");
    assert!(outcomes.solved > 0, "{outcomes:?}");
    assert!(outcomes.unsolvable > 0, "{outcomes:?}");
}

/// Asserts [`assert_covers_all`] plus a task where BFS is not cost-optimal,
/// so the sweep separates cost-aware from unit-cost search.
fn assert_covers_costs(outcomes: &Outcomes) {
    assert_covers_all(outcomes);
    assert!(outcomes.cost_matters > 0, "{outcomes:?}");
}

/// Default generator, costs 1 to 5.
#[test]
fn default_tasks_match_dijkstra_oracle() {
    assert_covers_costs(&sweep(RandomTaskParams::default(), 64));
}

/// Wider domains (invalid codes) and more operators.
#[test]
fn wide_domains_match_dijkstra_oracle() {
    let params = RandomTaskParams {
        num_vars: 5,
        max_domain: 5,
        num_ops: 12,
        ..RandomTaskParams::default()
    };

    assert_covers_all(&sweep(params, 64));
}

/// Conditional effects with one condition each.
#[test]
fn conditional_effects_match_dijkstra_oracle() {
    let params = RandomTaskParams {
        max_domain: 5,
        num_ops: 14,
        max_conditions: 1,
        ..RandomTaskParams::default()
    };

    assert_covers_costs(&sweep(params, 64));
}

/// An operator `name` from `x = from` to `x = to` with the given cost.
fn step(name: &str, from: u32, to: u32, cost: u32) -> Operator {
    Operator {
        name: name.into(),
        pre: vec![Fact::new(0, from)],
        eff: vec![Effect::new(Fact::new(0, to))],
        cost,
    }
}

/// A line `0 → 1 → 2 → 3` where two zero-cost operators form a cycle
/// between 1 and 2, and a zero-cost step reaches the goal.
fn zero_cost_line() -> Task {
    Task {
        variables: vec![Variable::new("x", 4)],
        operators: vec![
            step("pay", 0, 1, 3),
            step("forth", 1, 2, 0),
            step("back", 2, 1, 0),
            step("free", 2, 3, 0),
        ],
        initial: State(vec![0]),
        goal: vec![Fact::new(0, 3)],
        axioms: vec![],
    }
}

/// Zero-cost cycles and a zero-cost last step: optimum 3, no endless loop.
#[test]
fn zero_cost_cycles_and_steps() {
    assert_eq!(check_costs(&zero_cost_line()), Some(3));
}

/// Only zero-cost operators: cost 0, but the plan is not empty.
#[test]
fn all_zero_costs() {
    let mut task = zero_cost_line();
    task.operators[0].cost = 0;

    assert_eq!(check_costs(&task), Some(0));
    let plan = SymbolicTask::new(&task, SMALL)
        .and_then(|s| s.bidirectional_dijkstra())
        .expect("search fits")
        .expect("solvable");
    assert_eq!(plan.len(), 3);
}

/// Gripper with action costs: the cost optimum is 7, not the BFS plan.
#[test]
fn gripper_costs_is_cost_optimal() {
    assert_eq!(
        check_costs(&crate::fixtures::fixture("gripper-costs")),
        Some(7)
    );
}
