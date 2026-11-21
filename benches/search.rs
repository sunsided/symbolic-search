//! Benchmarks of the symbolic searches on the gripper-with-costs fixture
//! and on a larger random-walk task.
//!
//! Each iteration runs one full search on an already compiled task, so the
//! numbers measure images, preimages, bucket handling and reconstruction.

use criterion::{Criterion, criterion_group, criterion_main};
use sas_plus::fdr::SasFile;
use symbolic_search::{Config, Partitioning, Schedule, SymbaConfig, SymbolicTask};

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

/// [`Config::default`] with conjunctive clusters of up to
/// `cluster_node_limit` nodes (0: one cluster per conjunct) under
/// `schedule`.
fn conjunctive(cluster_node_limit: usize, schedule: Schedule) -> Config {
    Config {
        partitioning: Partitioning::Conjunctive {
            cluster_node_limit,
            schedule,
        },
        ..Config::default()
    }
}

/// The partitioning variants of [`partitionings`], by benchmark name.
fn partitioning_variants() -> [(&'static str, Config); 6] {
    [
        ("disjunctive_merged", Config::default()),
        (
            "disjunctive_per_operator",
            Config {
                merge_node_limit: 0,
                ..Config::default()
            },
        ),
        (
            "conjunctive_per_conjunct_greedy",
            conjunctive(0, Schedule::Greedy),
        ),
        (
            "conjunctive_per_conjunct_iwls95",
            conjunctive(0, Schedule::Iwls95),
        ),
        (
            "conjunctive_clustered_greedy",
            conjunctive(64, Schedule::Greedy),
        ),
        (
            "conjunctive_clustered_iwls95",
            conjunctive(64, Schedule::Iwls95),
        ),
    ]
}

/// Forward and backward Dijkstra under merged disjunctive groups, one
/// disjunctive relation per operator (to separate the cost of lost merging
/// from the cost of the conjuncts), and conjunctive clusters under the
/// greedy and the IWLS95 schedule.
fn partitionings(c: &mut Criterion) {
    let mut group = c.benchmark_group("gripper-costs-partitioning");
    for (name, config) in partitioning_variants() {
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

/// A random task with 14 variables and 48 operators whose goal is 6 facts of
/// the state a 60-step random walk reaches (optimal cost 19 at seed 0); the
/// same generator as the crate's `large_task` tuning helper.
fn random_walk_task(seed: u64) -> sas_plus::Task {
    let params = sas_plus::random::RandomTaskParams {
        num_vars: 14,
        max_domain: 4,
        num_ops: 48,
        ..sas_plus::random::RandomTaskParams::default()
    };
    let mut task = sas_plus::random::random_task(seed, params);
    let mut rng = seed.wrapping_mul(0x9E37_79B9_7F4A_7C15) | 1;
    let mut next = move |n: usize| {
        rng ^= rng << 13;
        rng ^= rng >> 7;
        rng ^= rng << 17;
        usize::try_from(rng % u64::try_from(n).unwrap()).unwrap()
    };

    let mut state = task.initial_state();
    for _ in 0..60 {
        let applicable: Vec<_> = task.applicable_operators(&state).collect();
        if applicable.is_empty() {
            break;
        }
        state = task.apply(applicable[next(applicable.len())], &state);
    }

    let mut vars: Vec<usize> = (0..task.variables.len()).collect();
    task.goal = (0..6)
        .map(|_| vars.remove(next(vars.len())))
        .map(|var| sas_plus::Fact::new(var, state.0[var]))
        .collect();

    task
}

/// Cost-optimal searches on the random-walk task: blind bidirectional,
/// `SymBA*` with default parameters, and `BDDA*` with merge-and-shrink and
/// canonical singleton PDBs.
fn random_walk(c: &mut Criterion) {
    let task = random_walk_task(0);
    let symbolic = SymbolicTask::new(&task, Config::for_task(&task)).expect("task compiles");
    let shrunk = symbolic.merge_and_shrink(1024).expect("m&s fits");
    let singles: Vec<Vec<usize>> = (0..task.variables.len()).map(|v| vec![v]).collect();
    let canonical = symbolic.canonical_pdbs(&singles).expect("pdbs fit");
    let mut group = c.benchmark_group("random-walk");
    group.sample_size(10);

    group.bench_function("bidirectional_dijkstra", |b| {
        b.iter(|| symbolic.bidirectional_dijkstra());
    });
    group.bench_function("symba_default", |b| {
        b.iter(|| symbolic.symba_with(&SymbaConfig::default()));
    });
    group.bench_function("astar_merge_and_shrink", |b| {
        b.iter(|| symbolic.astar(&shrunk));
    });
    group.bench_function("astar_canonical", |b| b.iter(|| symbolic.astar(&canonical)));
    group.finish();
}

criterion_group!(benches, searches, partitionings, random_walk);
criterion_main!(benches);
