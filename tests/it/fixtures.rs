//! Hand-encoded Fast Downward fixtures, copied from the `sas-plus` repository.

use sas_plus::Task;
use sas_plus::fdr::SasFile;
use symbolic_search::{Error, SymbolicTask};

use crate::{SMALL, check_against_oracle};

/// Parses `tests/fixtures/<name>/output.sas` into a task.
pub fn fixture(name: &str) -> Task {
    let path = format!(
        "{}/tests/fixtures/{name}/output.sas",
        env!("CARGO_MANIFEST_DIR")
    );
    let text = std::fs::read_to_string(&path).expect("fixture exists");

    SasFile::parse(&text).expect("fixture parses").to_task()
}

/// Gripper with unit costs: the known optimum is 5 steps.
#[test]
fn gripper_unit_is_optimal() {
    assert_eq!(check_against_oracle(&fixture("gripper-unit")), Some(5));
}

/// Gripper with action costs: BFS ignores costs, so the length matches the
/// unit-cost oracle, not the cost-optimal plan.
#[test]
fn gripper_costs_matches_unit_oracle() {
    assert!(check_against_oracle(&fixture("gripper-costs")).is_some());
}

/// Conditional effects and a `<none of those>` value.
#[test]
fn conditional_effects_match_oracle() {
    check_against_oracle(&fixture("conditional-effects"));
}

/// Derived variables are rejected with a typed error.
#[test]
fn derived_predicates_are_rejected() {
    let result = SymbolicTask::new(&fixture("derived-predicates"), SMALL);

    assert!(matches!(result, Err(Error::Axioms)));
}
