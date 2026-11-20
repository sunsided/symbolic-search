//! Capacities derived from the task, and automatic retries with a larger
//! manager on running out of nodes.

use symbolic_search::{Config, Error, SymbolicTask};

use crate::fixtures::fixture;
use crate::{SMALL, validated_cost};

/// Searches the gripper-costs fixture (optimum 7) from a node table that
/// is far too small for it: compiling and searching grow the manager.
#[test]
fn retries_grow_the_manager_until_the_search_fits() {
    let task = fixture("gripper-costs");
    let tiny = Config {
        node_capacity: 1 << 6,
        cache_capacity: 1 << 4,
        ..SMALL
    };

    let symbolic = SymbolicTask::new(&task, tiny).expect("compiling grows the table");
    let compiled = symbolic.node_capacity();
    assert!(compiled > tiny.node_capacity);

    let solution = symbolic.forward_dijkstra().expect("search grows the table");
    assert_eq!(
        solution.map(|s| validated_cost(&task, &s, "dijkstra")),
        Some(7)
    );
    assert!(symbolic.node_capacity() >= compiled);
    assert!(symbolic.node_capacity() <= SMALL.max_node_capacity);
}

/// A search that needs a larger table than the one it compiled in grows it,
/// and the grown table is kept for the next search.
#[test]
fn a_search_retry_is_remembered() {
    let task = fixture("gripper-costs");
    // Capacities that are not powers of two, so a table that just fits the
    // relations can be too small for the search.
    let grew = (64..4096).step_by(8).find_map(|nodes| {
        let config = Config {
            node_capacity: nodes,
            cache_capacity: 1 << 4,
            ..SMALL
        };
        let symbolic = SymbolicTask::new(&task, config).ok()?;
        let before = symbolic.node_capacity();
        let solution = symbolic.bidirectional_dijkstra().expect("retries fit");
        (symbolic.node_capacity() > before).then_some((symbolic, solution))
    });

    let (symbolic, solution) = grew.expect("some table fits the task but not the search");
    assert_eq!(solution.map(|s| s.cost), Some(7));
    let grown = symbolic.node_capacity();
    assert_eq!(symbolic.symba(1).unwrap().map(|s| s.cost), Some(7));
    assert!(symbolic.node_capacity() >= grown);
}

/// Heuristics built in a table too small for them are built in a grown
/// one, and heuristic searches and evaluation rebuild them there.
#[test]
fn heuristic_searches_retry_in_a_grown_table() {
    let task = fixture("gripper-costs");
    let grew = (64..4096).step_by(8).find_map(|nodes| {
        let config = Config {
            node_capacity: nodes,
            cache_capacity: 1 << 4,
            ..SMALL
        };
        let symbolic = SymbolicTask::new(&task, config).ok()?;
        let before = symbolic.node_capacity();
        let pdb = symbolic
            .canonical_pdbs(&[vec![0], vec![1]])
            .expect("retries fit");
        let solution = symbolic.astar(&pdb).expect("retries fit");
        (symbolic.node_capacity() > before).then_some((symbolic, pdb, solution))
    });

    let (symbolic, pdb, solution) = grew.expect("some table fits the task but not the heuristic");
    assert_eq!(solution.map(|s| s.cost), Some(7));
    let greedy = symbolic.greedy(&pdb).expect("fits").expect("solvable");
    assert_eq!(validated_cost(&task, &greedy, "greedy"), greedy.cost);
    let h = symbolic.evaluate(&pdb, &task.initial).expect("fits");
    let small = SymbolicTask::new(&task, SMALL).unwrap();
    let reference = small.canonical_pdbs(&[vec![0], vec![1]]).unwrap();
    assert_eq!(h, small.evaluate(&reference, &task.initial).unwrap());
}

/// Without room to grow, running out of nodes is an error.
#[test]
fn no_retry_past_the_cap() {
    let config = Config {
        node_capacity: 1 << 6,
        cache_capacity: 1 << 4,
        max_node_capacity: 1 << 6,
        ..SMALL
    };

    let result = SymbolicTask::new(&fixture("gripper-costs"), config);
    assert!(matches!(result, Err(Error::OutOfMemory)));
}

/// Derived capacities fit every solvable fixture without a retry.
#[test]
fn derived_capacities_fit_the_fixtures() {
    for name in ["gripper-unit", "gripper-costs", "conditional-effects"] {
        let task = fixture(name);
        let config = Config::for_task(&task);
        assert!(config.node_capacity <= 1 << 22);
        assert!(config.max_node_capacity >= config.node_capacity);

        let symbolic = SymbolicTask::new(&task, config).expect("compiles");
        let solution = symbolic.bidirectional_dijkstra().expect("fits");
        assert!(solution.is_some(), "{name} is solvable");
        assert_eq!(symbolic.node_capacity(), config.node_capacity, "{name}");
    }
}
