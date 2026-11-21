//! `BDDA*` and GBFS with symbolic PDBs and explicit heuristics, and the PDB
//! values against `sas_plus::Projection`.

use sas_plus::random::{RandomTaskParams, random_task};
use sas_plus::{Abstraction, Fact, Projection, State, Task, VarId};
use symbolic_search::{Error, Heuristic, SymbolicTask};

use crate::{CONFIGS, SMALL, oracle, validated_cost};

/// The facts of a complete state.
fn facts(state: &State) -> Vec<Fact> {
    state
        .0
        .iter()
        .enumerate()
        .map(|(var, &value)| Fact::new(var, value))
        .collect()
}

/// The perfect heuristic on the reachable states, as explicit estimates.
/// With `inconsistent`, states with an odd value sum get 0 instead, which
/// stays admissible but breaks consistency.
fn perfect(symbolic: &SymbolicTask, task: &Task, inconsistent: bool) -> Heuristic {
    let h_star = task.perfect_heuristic();
    let estimates: Vec<(Vec<Fact>, u64)> = oracle::reachable_states(task)
        .iter()
        .filter_map(|state| {
            let odd = state.0.iter().sum::<u32>() % 2 == 1;
            let h = if inconsistent && odd {
                0
            } else {
                h_star(state)?
            };
            Some((facts(state), h))
        })
        .collect();

    symbolic.heuristic(&estimates).expect("estimates fit")
}

/// The pattern used for PDB tests: the goal variables, or variable 0.
fn pattern(task: &Task) -> Vec<VarId> {
    let mut vars: Vec<VarId> = task.goal.iter().map(|f| f.var).collect();
    vars.sort_unstable();
    vars.dedup();
    if vars.is_empty() {
        vars.push(0);
    }

    vars
}

/// One singleton pattern per variable of `task`.
fn singletons(task: &Task) -> Vec<Vec<VarId>> {
    (0..task.variables.len()).map(|var| vec![var]).collect()
}

/// Checks the canonical heuristic of the singleton patterns and the
/// merge-and-shrink heuristic on every reachable state: at most `h*`, dead
/// only on real dead ends, and the canonical one at least every single
/// PDB.
pub fn check_canonical_values(task: &Task) {
    let symbolic = SymbolicTask::new(task, SMALL).expect("task compiles");
    let patterns = singletons(task);
    let canonical = symbolic.canonical_pdbs(&patterns).expect("pdbs fit");
    let shrunk = symbolic.merge_and_shrink(16).expect("m&s fits");
    let singles: Vec<Heuristic> = patterns
        .iter()
        .map(|p| symbolic.pdb(p).expect("pdb fits"))
        .collect();
    let h_star = task.perfect_heuristic();

    for state in oracle::reachable_states(task) {
        let m = symbolic.evaluate(&shrunk, &state).expect("fits");
        match m {
            Some(m) => assert!(
                h_star(&state).is_none_or(|star| m <= star),
                "m&s admissible"
            ),
            None => assert!(h_star(&state).is_none(), "m&s: only real dead ends"),
        }

        let h = symbolic.evaluate(&canonical, &state).expect("fits");
        match h {
            Some(h) => assert!(h_star(&state).is_none_or(|star| h <= star), "admissible"),
            None => assert!(h_star(&state).is_none(), "only real dead ends"),
        }

        for single in &singles {
            let s = symbolic.evaluate(single, &state).expect("fits");
            // `None` (dead) counts as infinity.
            assert!(h.is_none() || s.is_some_and(|s| s <= h.unwrap()));
        }
    }
}

/// Checks `BDDA*` with several heuristics and GBFS on `task`; returns the
/// optimal cost.
pub fn check_heuristic_search(task: &Task) -> Option<u64> {
    let expected = oracle::cheapest_plan_cost(task);
    for config in CONFIGS {
        let symbolic = SymbolicTask::new(task, config).expect("task compiles");
        let pdb = symbolic.pdb(&pattern(task)).expect("pdb fits");
        let canonical = symbolic
            .canonical_pdbs(&singletons(task))
            .expect("pdbs fit");
        let shrunk = symbolic.merge_and_shrink(16).expect("m&s fits");
        let runs = [
            ("astar with merge-and-shrink", symbolic.astar(&shrunk)),
            ("astar with pdb", symbolic.astar(&pdb)),
            ("astar with canonical pdbs", symbolic.astar(&canonical)),
            (
                "astar with h*",
                symbolic.astar(&perfect(&symbolic, task, false)),
            ),
            (
                "astar inconsistent",
                symbolic.astar(&perfect(&symbolic, task, true)),
            ),
        ];
        for (what, plan) in runs {
            let cost = plan
                .expect("search fits")
                .map(|plan| validated_cost(task, &plan, what));
            assert_eq!(cost, expected, "{what} is optimal");
        }

        // The PDB only prunes real dead ends, so GBFS is complete with it.
        let greedy = symbolic.greedy(&pdb).expect("search fits");
        let cost = greedy.map(|plan| validated_cost(task, &plan, "greedy"));
        assert_eq!(cost.is_some(), expected.is_some(), "greedy finds a plan");
    }

    expected
}

/// Checks the PDB of [`pattern`] on every reachable state against the
/// explicit projection: equal without conditional effects, between the
/// projection and `h*` with them.
pub fn check_pdb_values(task: &Task) {
    let symbolic = SymbolicTask::new(task, SMALL).expect("task compiles");
    let pattern = pattern(task);
    let pdb = symbolic.pdb(&pattern).expect("pdb fits");
    let projection = Projection::new(task, pattern);
    let distances = projection.lts().goal_distances(&task.costs());
    let h_star = task.perfect_heuristic();
    let exact = task
        .operators
        .iter()
        .all(|op| op.eff.iter().all(|e| e.conditions.is_empty()));

    for state in oracle::reachable_states(task) {
        let symbolic_h = symbolic.evaluate(&pdb, &state).expect("evaluation fits");
        let explicit_h = distances[projection.abstract_state(&state).expect("projected")];
        if exact {
            assert_eq!(symbolic_h, explicit_h, "pdb value of {state:?}");
        } else {
            // `None` (dead) counts as infinity in both orders.
            assert!(symbolic_h.is_none_or(|h| explicit_h.is_some_and(|e| e <= h)));
            match symbolic_h {
                Some(h) => assert!(h_star(&state).is_none_or(|star| h <= star), "admissible"),
                None => assert!(h_star(&state).is_none(), "only real dead ends"),
            }
        }
    }
}

/// Default random tasks: `BDDA*` is optimal, PDB values match.
#[test]
fn heuristic_search_on_random_tasks() {
    let mut solved = 0;
    for seed in 0..48 {
        let task = random_task(seed, RandomTaskParams::default());
        check_pdb_values(&task);
        check_canonical_values(&task);
        solved += usize::from(check_heuristic_search(&task).is_some_and(|c| c > 0));
    }

    assert!(solved > 0);
}

/// Conditional effects: `BDDA*` optimal, PDB between projection and `h*`.
#[test]
fn heuristic_search_with_conditional_effects() {
    let params = RandomTaskParams {
        max_domain: 5,
        max_conditions: 2,
        ..RandomTaskParams::default()
    };
    for seed in 0..48 {
        let task = random_task(seed, params);
        check_pdb_values(&task);
        check_canonical_values(&task);
        check_heuristic_search(&task);
    }
}

/// A heuristic from one task is rejected by another.
#[test]
fn foreign_heuristic_is_rejected() {
    let task = random_task(1, RandomTaskParams::default());
    let first = SymbolicTask::new(&task, SMALL).unwrap();
    let second = SymbolicTask::new(&task, SMALL).unwrap();
    let pdb = first.pdb(&[0]).unwrap();

    assert!(matches!(second.astar(&pdb), Err(Error::ForeignHeuristic)));
}

/// Bad patterns and facts are typed errors.
#[test]
fn invalid_inputs_are_rejected() {
    let task = random_task(1, RandomTaskParams::default());
    let symbolic = SymbolicTask::new(&task, SMALL).unwrap();
    let domain = task.variables[0].domain_size;

    assert!(matches!(
        symbolic.pdb(&[99]),
        Err(Error::UnknownVariable(99))
    ));
    let bad = Fact::new(0, domain);
    assert!(matches!(
        symbolic.heuristic(&[(vec![bad], 1)]),
        Err(Error::InvalidFact(f)) if f == bad
    ));
}
