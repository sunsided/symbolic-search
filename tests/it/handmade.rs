//! Small hand-built tasks for edge cases the generators do not hit.

use sas_plus::{Effect, Fact, Operator, State, Task, Variable};

use crate::{check_against_oracle, symbolic_plans};

/// An operator named `name` with the given preconditions and effects, cost 1.
fn operator(name: &str, pre: Vec<Fact>, eff: Vec<Effect>) -> Operator {
    Operator {
        name: name.into(),
        pre,
        eff,
        cost: 1,
    }
}

/// An effect setting `var = value` when all `conditions` hold.
fn when(conditions: Vec<Fact>, var: usize, value: u32) -> Effect {
    Effect {
        conditions,
        fact: Fact::new(var, value),
    }
}

/// Two conditional effects on one variable fire together; the later one wins.
/// Reaching value 2 needs `set` (which ends at 2, not 1) and then nothing else.
#[test]
fn later_conditional_effect_wins() {
    let task = Task {
        variables: vec![Variable::new("x", 3), Variable::new("y", 2)],
        operators: vec![
            operator(
                "set",
                vec![],
                vec![when(vec![Fact::new(1, 0)], 0, 1), when(vec![], 0, 2)],
            ),
            operator("flip", vec![Fact::new(0, 1)], vec![when(vec![], 1, 1)]),
        ],
        initial: State(vec![0, 0]),
        goal: vec![Fact::new(0, 2)],
        axioms: vec![],
    };

    assert_eq!(check_against_oracle(&task), Some(1));
}

/// The goal holds initially: both searches return the empty plan.
#[test]
fn goal_in_initial_state_gives_empty_plan() {
    let task = Task {
        variables: vec![Variable::new("x", 3)],
        operators: vec![operator("inc", vec![], vec![when(vec![], 0, 1)])],
        initial: State(vec![2]),
        goal: vec![Fact::new(0, 2)],
        axioms: vec![],
    };

    assert_eq!(symbolic_plans(&task), (Some(vec![]), Some(vec![])));
}

/// A single-value variable takes zero bits and must not break anything.
#[test]
fn single_value_variable_is_encoded_with_no_bits() {
    let task = Task {
        variables: vec![Variable::new("const", 1), Variable::new("x", 2)],
        operators: vec![operator(
            "go",
            vec![Fact::new(0, 0)],
            vec![when(vec![], 0, 0), when(vec![], 1, 1)],
        )],
        initial: State(vec![0, 0]),
        goal: vec![Fact::new(0, 0), Fact::new(1, 1)],
        axioms: vec![],
    };

    assert_eq!(check_against_oracle(&task), Some(1));
}

/// No operator: unsolvable unless the goal holds initially.
#[test]
fn no_operators_is_unsolvable() {
    let task = Task {
        variables: vec![Variable::new("x", 5)],
        operators: vec![],
        initial: State(vec![0]),
        goal: vec![Fact::new(0, 4)],
        axioms: vec![],
    };

    assert_eq!(symbolic_plans(&task), (None, None));
}
