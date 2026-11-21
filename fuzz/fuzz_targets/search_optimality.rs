//! Fuzzes the cost arithmetic of the cost-optimal searches: random valid
//! tasks whose operator costs are arbitrary `u32` values, zero and
//! `u32::MAX` included.
//!
//! Dijkstra in every direction, `SymBA*` on every hierarchy and `BDDA*`
//! with a PDB, canonical PDBs and merge-and-shrink must all return a valid
//! plan whose cost equals the explicit optimum `h*` of the initial state.
//! The property tests cover the same searches with costs 1 to 5 only.

#![no_main]
#![forbid(unsafe_code)]

use libfuzzer_sys::arbitrary::{Result, Unstructured};
use libfuzzer_sys::fuzz_target;
use sas_plus::Task;
use sas_plus::random::{RandomTaskParams, random_task};
use symbolic_search::{
    AbstractionHierarchy, Config, Error, Partitioning, Solution, SymbaConfig, SymbolicTask,
    VariableOrder,
};

/// Node limits that keep one run fast and far below libFuzzer's memory
/// limit; running out of nodes is an `Err`, not a finding.
const LIMITS: Config = Config {
    node_capacity: 1 << 13,
    cache_capacity: 1 << 11,
    max_node_capacity: 1 << 15,
    merge_node_limit: 1 << 10,
    variable_order: VariableOrder::Gamer,
    partitioning: Partitioning::Disjunctive,
};

/// Generator parameters small enough for the explicit `h*`: up to four
/// variables of domain up to 5, ten operators and two effect conditions,
/// no derived variables.
fn params(u: &mut Unstructured) -> Result<RandomTaskParams> {
    Ok(RandomTaskParams {
        num_vars: u.int_in_range(1..=4)?,
        max_domain: u.int_in_range(2..=5)?,
        num_ops: u.int_in_range(0..=10)?,
        max_cost: 1,
        max_conditions: u.int_in_range(0..=2)?,
        num_derived: 0,
    })
}

/// A random task with every operator cost replaced by an arbitrary `u32`.
fn task(u: &mut Unstructured) -> Result<Task> {
    let mut task = random_task(u.arbitrary()?, params(u)?);
    for operator in &mut task.operators {
        operator.cost = u.arbitrary()?;
    }

    Ok(task)
}

/// `SymBA*` that abstracts at once and resumes abstract searches after
/// every expansion, on `hierarchy`.
fn symba(hierarchy: AbstractionHierarchy) -> SymbaConfig {
    SymbaConfig {
        frontier_node_limit: 1,
        abstract_node_limit: 64,
        abstract_slice: 1,
        max_expansions: None,
        time_limit: None,
        hierarchy,
    }
}

/// The searches that must be cost-optimal, by name.
fn runs(
    task: &Task,
    symbolic: &SymbolicTask,
) -> Vec<(&'static str, core::result::Result<Option<Solution>, Error>)> {
    let goal_vars: Vec<usize> = task.goal.iter().map(|fact| fact.var).collect();
    let singletons: Vec<Vec<usize>> = (0..task.variables.len()).map(|var| vec![var]).collect();
    let mut runs = vec![
        ("forward dijkstra", symbolic.forward_dijkstra()),
        ("backward dijkstra", symbolic.backward_dijkstra()),
        ("bidirectional dijkstra", symbolic.bidirectional_dijkstra()),
        ("symba, never abstracting", symbolic.symba(usize::MAX)),
        (
            "symba, dropping variables",
            symbolic.symba_with(&symba(AbstractionHierarchy::DropVariables)),
        ),
        (
            "symba, dropping bits",
            symbolic.symba_with(&symba(AbstractionHierarchy::DropBits)),
        ),
        (
            "symba, merge-and-shrink",
            symbolic.symba_with(&symba(AbstractionHierarchy::MergeAndShrink {
                node_limit: 16,
            })),
        ),
    ];
    let heuristics = [
        ("astar with pdb", symbolic.pdb(&goal_vars)),
        (
            "astar with canonical pdbs",
            symbolic.canonical_pdbs(&singletons),
        ),
        ("astar with merge-and-shrink", symbolic.merge_and_shrink(16)),
    ];
    for (what, heuristic) in heuristics {
        if let Ok(heuristic) = heuristic {
            runs.push((what, symbolic.astar(&heuristic)));
        }
    }

    runs
}

/// Builds a task from `data` and checks every cost-optimal search on it.
fn run(data: &[u8]) -> Result<()> {
    let mut u = Unstructured::new(data);
    let task = task(&mut u)?;
    let Ok(symbolic) = SymbolicTask::new(&task, LIMITS) else {
        return Ok(());
    };
    let optimum = task.perfect_heuristic()(&task.initial_state());

    for (what, run) in runs(&task, &symbolic) {
        // Running out of nodes is allowed; a wrong answer is not.
        let Ok(found) = run else { continue };
        if let Some(solution) = &found {
            assert_eq!(
                task.validate_plan(&solution.plan),
                Some(solution.cost),
                "{what}: plan {:?} is invalid or mis-costed",
                solution.plan
            );
        }

        assert_eq!(found.map(|s| s.cost), optimum, "{what} is optimal");
    }

    Ok(())
}

fuzz_target!(|data: &[u8]| {
    // Running out of input mid-task only ends the run early.
    let _ = run(data);
});
