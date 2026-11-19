//! Seed sweeps over `sas_plus::random` tasks.

use sas_plus::random::{RandomTaskParams, random_task};

use crate::check_against_oracle;

/// Tally of a seed sweep, to prove the sweep covers every outcome.
#[derive(Debug, Default)]
struct Outcomes {
    /// Initial state is already a goal state.
    trivial: usize,
    /// Solvable with at least two operators.
    deep: usize,
    /// No goal state is reachable.
    unsolvable: usize,
}

/// Checks `seeds` tasks generated with `params` and returns the tally.
fn sweep(params: RandomTaskParams, seeds: u64) -> Outcomes {
    let mut outcomes = Outcomes::default();
    for seed in 0..seeds {
        match check_against_oracle(&random_task(seed, params)) {
            Some(0) => outcomes.trivial += 1,
            Some(n) if n >= 2 => outcomes.deep += 1,
            Some(_) => {}
            None => outcomes.unsolvable += 1,
        }
    }

    outcomes
}

/// Asserts that a sweep saw trivial, deep and unsolvable tasks.
fn assert_covers_all(outcomes: &Outcomes) {
    assert!(outcomes.trivial > 0, "{outcomes:?}");
    assert!(outcomes.deep > 0, "{outcomes:?}");
    assert!(outcomes.unsolvable > 0, "{outcomes:?}");
}

/// Default generator: four variables of domain up to 3.
#[test]
fn default_tasks_match_oracle() {
    assert_covers_all(&sweep(RandomTaskParams::default(), 64));
}

/// Domains up to 5 exercise invalid codes of the binary encoding.
#[test]
fn wide_domains_match_oracle() {
    let params = RandomTaskParams {
        num_vars: 5,
        max_domain: 5,
        num_ops: 12,
        ..RandomTaskParams::default()
    };

    assert_covers_all(&sweep(params, 64));
}

/// Conditional effects with up to two conditions each.
#[test]
fn conditional_effects_match_oracle() {
    let params = RandomTaskParams {
        max_domain: 5,
        max_conditions: 2,
        ..RandomTaskParams::default()
    };

    assert_covers_all(&sweep(params, 64));
}
