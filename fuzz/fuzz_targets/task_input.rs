//! Fuzzes what a caller hands to `symbolic-search`: arbitrary, often invalid
//! tasks, then patterns, explicit estimates and states for the heuristic API.
//!
//! Every call must return `Ok` or `Err` and never panic. Whatever plan comes
//! back must pass [`Task::validate_plan`] with the reported cost, the
//! cost-optimal searches must reach the explicit optimum `h*`, and a PDB
//! must never overestimate it.

#![no_main]
#![forbid(unsafe_code)]

use libfuzzer_sys::arbitrary::{Result, Unstructured};
use libfuzzer_sys::fuzz_target;
use sas_plus::{Effect, Fact, Operator, State, Task, Variable};
use symbolic_search::{
    Config, Error, Heuristic, Partitioning, Schedule, Solution, SymbolicTask, VariableOrder,
};

/// Most variables in a fuzzed task; with [`MAX_DOMAIN`] the explicit `h*`
/// stays cheap (at most 6⁴ states).
const MAX_VARS: usize = 4;

/// Largest domain size of a fuzzed variable. Sizes 0 and 1 are allowed on
/// purpose: 0 makes the task invalid, 1 needs no bits.
const MAX_DOMAIN: u32 = 6;

/// Most operators in a fuzzed task.
const MAX_OPS: usize = 6;

/// Most facts in one precondition, effect list, condition list, goal,
/// pattern or estimate.
const MAX_FACTS: usize = 3;

/// Node limits that keep one run fast and far below libFuzzer's memory
/// limit; running out of nodes is an `Err`, not a finding.
const LIMITS: Config = Config {
    node_capacity: 1 << 12,
    cache_capacity: 1 << 10,
    max_node_capacity: 1 << 14,
    merge_node_limit: 1 << 10,
    variable_order: VariableOrder::Gamer,
    partitioning: Partitioning::Disjunctive,
};

/// A variable id up to one past the last variable, so unknown variables
/// show up as often as valid ones at the edge.
fn var_id(u: &mut Unstructured, num_vars: usize) -> Result<usize> {
    u.int_in_range(0..=num_vars)
}

/// A fact on a variable of `domains` (or one past them), with a value up to
/// one past the variable's domain.
fn fact(u: &mut Unstructured, domains: &[u32]) -> Result<Fact> {
    let var = var_id(u, domains.len())?;
    let domain = domains.get(var).copied().unwrap_or(MAX_DOMAIN);
    let value = u.int_in_range(0..=domain)?;

    Ok(Fact::new(var, value))
}

/// Up to [`MAX_FACTS`] facts from [`fact`].
fn facts(u: &mut Unstructured, domains: &[u32]) -> Result<Vec<Fact>> {
    let len = u.int_in_range(0..=MAX_FACTS)?;

    (0..len).map(|_| fact(u, domains)).collect()
}

/// A variable with a domain of size 0 to [`MAX_DOMAIN`], derived (an axiom
/// layer) in about one case of sixteen.
fn variable(u: &mut Unstructured, index: usize) -> Result<Variable> {
    let mut variable = Variable::new(format!("v{index}"), u.int_in_range(0..=MAX_DOMAIN)?);
    if u.ratio(1, 16)? {
        variable.axiom_layer = Some(0);
    }

    Ok(variable)
}

/// An operator with arbitrary facts, conditional effects and any `u32`
/// cost.
fn operator(u: &mut Unstructured, domains: &[u32], index: usize) -> Result<Operator> {
    let pre = facts(u, domains)?;
    let effects = u.int_in_range(0..=MAX_FACTS)?;
    let eff = (0..effects)
        .map(|_| {
            Ok(Effect {
                conditions: facts(u, domains)?,
                fact: fact(u, domains)?,
            })
        })
        .collect::<Result<_>>()?;

    Ok(Operator {
        name: format!("op{index}"),
        pre,
        eff,
        cost: u.arbitrary()?,
    })
}

/// An initial state that has the right length in most cases, with values
/// up to one past each domain.
fn initial_state(u: &mut Unstructured, domains: &[u32]) -> Result<State> {
    let len = if u.ratio(1, 16)? {
        u.int_in_range(0..=MAX_VARS + 1)?
    } else {
        domains.len()
    };
    let values = (0..len)
        .map(|var| {
            let domain = domains.get(var).copied().unwrap_or(MAX_DOMAIN);
            u.int_in_range(0..=domain)
        })
        .collect::<Result<_>>()?;

    Ok(State(values))
}

/// A task built from fuzz input; it may break every rule
/// [`Task::validate`] checks.
fn task(u: &mut Unstructured) -> Result<Task> {
    let num_vars = u.int_in_range(0..=MAX_VARS)?;
    let variables: Vec<Variable> = (0..num_vars)
        .map(|index| variable(u, index))
        .collect::<Result<_>>()?;
    let domains: Vec<u32> = variables.iter().map(|v| v.domain_size).collect();
    let num_ops = u.int_in_range(0..=MAX_OPS)?;
    let operators = (0..num_ops)
        .map(|index| operator(u, &domains, index))
        .collect::<Result<_>>()?;

    Ok(Task {
        variables,
        operators,
        initial: initial_state(u, &domains)?,
        goal: facts(u, &domains)?,
        axioms: vec![],
    })
}

/// [`LIMITS`] with a fuzzed variable order and partitioning.
fn config(u: &mut Unstructured) -> Result<Config> {
    let variable_order = if u.arbitrary()? {
        VariableOrder::Gamer
    } else {
        VariableOrder::Task
    };
    let partitioning = match u.int_in_range(0..=2_u8)? {
        0 => Partitioning::Disjunctive,
        1 => Partitioning::Conjunctive {
            cluster_node_limit: 0,
            schedule: Schedule::Greedy,
        },
        _ => Partitioning::Conjunctive {
            cluster_node_limit: 64,
            schedule: Schedule::Iwls95,
        },
    };

    Ok(Config {
        variable_order,
        partitioning,
        ..LIMITS
    })
}

/// Asserts that a found plan is valid and costs what the search reports.
fn assert_valid(task: &Task, solution: &Solution, what: &str) {
    assert_eq!(
        task.validate_plan(&solution.plan),
        Some(solution.cost),
        "{what}: plan {:?} is invalid or its cost is wrong",
        solution.plan
    );
}

/// Runs BFS and the Dijkstra searches: every plan is valid, BFS and
/// Dijkstra agree on solvability, and Dijkstra costs equal `optimum`.
/// Searches that run out of nodes are skipped.
fn check_searches(task: &Task, symbolic: &SymbolicTask, optimum: Option<u64>) {
    if let Ok(bfs) = symbolic.forward_bfs() {
        if let Some(solution) = &bfs {
            assert_valid(task, solution, "forward bfs");
        }

        assert_eq!(bfs.is_some(), optimum.is_some(), "bfs solvability");
    }

    let runs = [
        ("forward dijkstra", symbolic.forward_dijkstra()),
        ("backward dijkstra", symbolic.backward_dijkstra()),
        ("bidirectional dijkstra", symbolic.bidirectional_dijkstra()),
    ];
    for (what, run) in runs {
        let Ok(found) = run else { continue };
        if let Some(solution) = &found {
            assert_valid(task, solution, what);
        }

        assert_eq!(found.map(|s| s.cost), optimum, "{what} is optimal");
    }
}

/// Asserts that `heuristic` never overestimates `optimum` in the initial
/// state and calls it dead only when it is.
fn assert_admissible(
    symbolic: &SymbolicTask,
    heuristic: &Heuristic,
    initial: &State,
    optimum: Option<u64>,
) {
    let Ok(h) = symbolic.evaluate(heuristic, initial) else {
        return;
    };

    match (h, optimum) {
        (Some(h), Some(star)) => assert!(h <= star, "h = {h} exceeds h* = {star}"),
        (None, Some(star)) => panic!("dead end reported, but h* = {star}"),
        _ => {}
    }
}

/// Feeds fuzzed patterns, estimates and states to the heuristic API. A PDB
/// must be admissible; everything else only must not panic.
fn probe_heuristics(
    u: &mut Unstructured,
    task: &Task,
    symbolic: &SymbolicTask,
    optimum: Option<u64>,
) -> Result<()> {
    let domains: Vec<u32> = task.variables.iter().map(|v| v.domain_size).collect();
    let len = u.int_in_range(0..=MAX_FACTS)?;
    let pattern = (0..len)
        .map(|_| var_id(u, domains.len()))
        .collect::<Result<Vec<_>>>()?;
    if let Ok(pdb) = symbolic.pdb(&pattern) {
        assert_admissible(symbolic, &pdb, &task.initial_state(), optimum);
    }

    let estimates = (0..u.int_in_range(0..=MAX_FACTS)?)
        .map(|_| Ok((facts(u, &domains)?, u.arbitrary()?)))
        .collect::<Result<Vec<(Vec<Fact>, u64)>>>()?;
    if let Ok(explicit) = symbolic.heuristic(&estimates) {
        let state = initial_state(u, &domains)?;
        // Arbitrary estimates need not be admissible; only no panic counts.
        let _ = symbolic.evaluate(&explicit, &state);
    }

    Ok(())
}

/// Builds a task and a config from `data`, then exercises the API on it.
fn run(data: &[u8]) -> Result<()> {
    let mut u = Unstructured::new(data);
    let task = task(&mut u)?;
    let config = config(&mut u)?;

    let symbolic = match SymbolicTask::new(&task, config) {
        Ok(symbolic) => symbolic,
        Err(Error::OutOfMemory | Error::InvalidTask(_) | Error::Axioms) => return Ok(()),
        Err(other) => panic!("unexpected compile error {other:?}"),
    };

    let optimum = task.perfect_heuristic()(&task.initial_state());
    check_searches(&task, &symbolic, optimum);

    probe_heuristics(&mut u, &task, &symbolic, optimum)
}

fuzz_target!(|data: &[u8]| {
    // Running out of input mid-task only ends the run early.
    let _ = run(data);
});
