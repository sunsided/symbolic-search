//! Benchmarks of the symbolic searches on the gripper-with-costs fixture.
//!
//! Each iteration runs one full search on an already compiled task, so the
//! numbers measure images, preimages, bucket handling and reconstruction.

use criterion::{Criterion, criterion_group, criterion_main};
use sas_plus::fdr::SasFile;
use symbolic_search::{Config, Partitioning, SymbaConfig, SymbolicTask};

/// Compiles the gripper-with-costs fixture with merged disjunctive groups.
fn gripper() -> SymbolicTask {
    gripper_with(Config::default())
}

/// Compiles the gripper-with-costs fixture with `base` and small tables.
fn gripper_with(base: Config) -> SymbolicTask {
    let path = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/gripper-costs/output.sas"
    );
    let text = std::fs::read_to_string(path).expect("fixture exists");
    let task = SasFile::parse(&text).expect("fixture parses").to_task();
    let config = Config {
        node_capacity: 1 << 16,
        cache_capacity: 1 << 14,
        ..base
    };

    SymbolicTask::new(&task, config).expect("task compiles")
}

/// One benchmark per search.
fn searches(c: &mut Criterion) {
    let symbolic = gripper();
    let pdb = symbolic.pdb(&[0, 1]).expect("pdb fits");
    let mut group = c.benchmark_group("gripper-costs");

    group.bench_function("forward_bfs", |b| b.iter(|| symbolic.forward_bfs()));
    group.bench_function("forward_dijkstra", |b| {
        b.iter(|| symbolic.forward_dijkstra());
    });
    group.bench_function("bidirectional_dijkstra", |b| {
        b.iter(|| symbolic.bidirectional_dijkstra());
    });
    group.bench_function("astar_pdb", |b| b.iter(|| symbolic.astar(&pdb)));
    group.bench_function("symba", |b| b.iter(|| symbolic.symba(64)));
    let resuming = SymbaConfig {
        frontier_node_limit: 64,
        abstract_node_limit: 64,
        abstract_slice: 1,
        ..SymbaConfig::default()
    };
    group.bench_function("symba_resuming", |b| {
        b.iter(|| symbolic.symba_with(&resuming));
    });
    group.finish();
}

/// Forward and bidirectional Dijkstra under merged disjunctive groups, one
/// disjunctive relation per operator (to separate the cost of lost merging
/// from the cost of the conjuncts), and conjunctive clusters.
fn partitionings(c: &mut Criterion) {
    let variants = [
        ("disjunctive_merged", Config::default()),
        (
            "disjunctive_per_operator",
            Config {
                merge_node_limit: 0,
                ..Config::default()
            },
        ),
        (
            "conjunctive_per_conjunct",
            Config {
                partitioning: Partitioning::Conjunctive {
                    cluster_node_limit: 0,
                },
                ..Config::default()
            },
        ),
        (
            "conjunctive_clustered",
            Config {
                partitioning: Partitioning::Conjunctive {
                    cluster_node_limit: 64,
                },
                ..Config::default()
            },
        ),
    ];
    let mut group = c.benchmark_group("gripper-costs-partitioning");
    for (name, config) in variants {
        let symbolic = gripper_with(config);
        group.bench_function(format!("{name}/forward_dijkstra"), |b| {
            b.iter(|| symbolic.forward_dijkstra());
        });
        group.bench_function(format!("{name}/backward_dijkstra"), |b| {
            b.iter(|| symbolic.backward_dijkstra());
        });
    }

    group.finish();
}

criterion_group!(benches, searches, partitionings);
criterion_main!(benches);
