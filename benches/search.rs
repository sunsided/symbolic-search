//! Benchmarks of the symbolic searches on the gripper-with-costs fixture.
//!
//! Each iteration runs one full search on an already compiled task, so the
//! numbers measure images, preimages, bucket handling and reconstruction.

use criterion::{Criterion, criterion_group, criterion_main};
use sas_plus::fdr::SasFile;
use symbolic_search::{Config, SymbolicTask};

/// Compiles the gripper-with-costs fixture.
fn gripper() -> SymbolicTask {
    let path = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/gripper-costs/output.sas"
    );
    let text = std::fs::read_to_string(path).expect("fixture exists");
    let task = SasFile::parse(&text).expect("fixture parses").to_task();
    let config = Config {
        node_capacity: 1 << 16,
        cache_capacity: 1 << 14,
        ..Config::default()
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
    group.finish();
}

criterion_group!(benches, searches);
criterion_main!(benches);
