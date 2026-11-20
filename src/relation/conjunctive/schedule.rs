//! Quantification schedules: the order in which conjuncts enter the
//! relational product.
//!
//! The smoothing variables are the affected variables of the operator,
//! whose current bits the image quantifies. Both schedules pick conjuncts
//! one at a time from the remaining set `Q`, after the precondition
//! conjunct on untouched variables.
//!
//! **Greedy.** Most retired smoothing variables (in the conjunct's support
//! and in no other remaining conjunct), ties to the smaller support, then
//! to the lower variable.
//!
//! **IWLS95** (Ranjan, Aziz, Brayton, Plessier and Pixley 1995). For a
//! candidate `c`:
//!
//! - `v`: smoothing variables that `c` retires;
//! - `w`: smoothing variables in the support of `c`;
//! - `x`: smoothing variables not yet quantified (in the support of some
//!   conjunct of `Q`);
//! - `y`: next-state variables `c` introduces;
//! - `z`: next-state variables not yet introduced (owned by `Q`);
//! - `m`: the largest diagram level of a smoothing variable of `c`, and `M`
//!   the largest over `Q`.
//!
//! The benefit is `W₁·v/w + W₂·w/x + W₃·y/z + W₄·m/M` with the paper's
//! weights `(2, 1, 1, 1)`; the largest benefit goes next, ties as in the
//! greedy order. `0/0` counts as 0. Every per-variable conjunct introduces
//! exactly its own next-state variable, so the `y/z` term is the same for
//! all candidates here and only the other three decide.

use std::collections::BTreeSet;

use sas_plus::VarId;

use crate::Schedule;
use crate::encoding::{Encoding, Frame};
use crate::relation::conjunctive::Conjunct;

/// The IWLS95 weights `W₁..W₄` from the paper's experiments.
const WEIGHTS: [f64; 4] = [2.0, 1.0, 1.0, 1.0];

/// Orders `conjuncts` by `schedule`. The precondition conjunct on untouched
/// variables, when present, is first in the input and stays first.
pub(super) fn order<S>(
    encoding: &Encoding,
    mut conjuncts: Vec<Conjunct<S>>,
    schedule: Schedule,
) -> Vec<Conjunct<S>> {
    let smoothing: BTreeSet<VarId> = conjuncts.iter().filter_map(|c| c.owner).collect();
    let mut ordered = Vec::with_capacity(conjuncts.len());
    if conjuncts.first().is_some_and(|c| c.owner.is_none()) {
        ordered.push(conjuncts.remove(0));
    }

    while !conjuncts.is_empty() {
        let next = match schedule {
            Schedule::Greedy => greedy_pick(&conjuncts, &smoothing),
            Schedule::Iwls95 => iwls95_pick(encoding, &conjuncts, &smoothing),
        };
        ordered.push(conjuncts.remove(next));
    }

    ordered
}

/// Index of the conjunct the greedy schedule takes next.
fn greedy_pick<S>(conjuncts: &[Conjunct<S>], smoothing: &BTreeSet<VarId>) -> usize {
    (0..conjuncts.len())
        .min_by_key(|&i| {
            (
                core::cmp::Reverse(retired_by(conjuncts, i, smoothing)),
                conjuncts[i].support.len(),
                conjuncts[i].owner,
            )
        })
        .expect("conjuncts is not empty")
}

/// Index of the conjunct with the largest IWLS95 benefit, ties as in
/// [`greedy_pick`].
fn iwls95_pick<S>(
    encoding: &Encoding,
    conjuncts: &[Conjunct<S>],
    smoothing: &BTreeSet<VarId>,
) -> usize {
    let benefits: Vec<f64> = (0..conjuncts.len())
        .map(|i| iwls95_benefit(encoding, conjuncts, i, smoothing))
        .collect();

    (0..conjuncts.len())
        .min_by(|&i, &j| {
            benefits[j]
                .total_cmp(&benefits[i])
                .then_with(|| conjuncts[i].support.len().cmp(&conjuncts[j].support.len()))
                .then_with(|| conjuncts[i].owner.cmp(&conjuncts[j].owner))
        })
        .expect("conjuncts is not empty")
}

/// The IWLS95 benefit of taking `conjuncts[index]` next (see the module
/// docs).
fn iwls95_benefit<S>(
    encoding: &Encoding,
    conjuncts: &[Conjunct<S>],
    index: usize,
    smoothing: &BTreeSet<VarId>,
) -> f64 {
    let level = |c: &Conjunct<S>| max_level(encoding, c, smoothing);
    let top = conjuncts.iter().map(level).max().unwrap_or(0);
    let open = conjuncts
        .iter()
        .flat_map(|c| c.support.iter().copied().chain(c.owner))
        .filter(|var| smoothing.contains(var))
        .collect::<BTreeSet<VarId>>()
        .len();
    let introduced_later = conjuncts.iter().filter(|c| c.owner.is_some()).count();

    let c = &conjuncts[index];
    let w = c
        .support
        .iter()
        .filter(|var| smoothing.contains(var))
        .count();
    let terms = [
        ratio(retired_by(conjuncts, index, smoothing), w),
        ratio(w, open),
        ratio(usize::from(c.owner.is_some()), introduced_later),
        ratio(level(c), top),
    ];

    terms.iter().zip(WEIGHTS).map(|(t, w)| t * w).sum()
}

/// `a / b` as a float, 0 when `b` is 0.
#[expect(
    clippy::cast_precision_loss,
    reason = "counts and levels are far below 2^52"
)]
fn ratio(a: usize, b: usize) -> f64 {
    if b == 0 { 0.0 } else { a as f64 / b as f64 }
}

/// The largest diagram level of a current bit of a smoothing variable in
/// the support of `conjunct`, 0 when there is none.
fn max_level<S>(encoding: &Encoding, conjunct: &Conjunct<S>, smoothing: &BTreeSet<VarId>) -> usize {
    conjunct
        .support
        .iter()
        .filter(|var| smoothing.contains(var))
        .flat_map(|&var| encoding.diagram_vars(var, Frame::Current))
        .map(|level| usize::try_from(level).expect("a diagram level fits usize"))
        .max()
        .unwrap_or(0)
}

/// How many smoothing variables would no longer be mentioned by any
/// remaining conjunct after taking `conjuncts[index]`.
fn retired_by<S>(conjuncts: &[Conjunct<S>], index: usize, smoothing: &BTreeSet<VarId>) -> usize {
    let mentioned_elsewhere = |var: &VarId| {
        conjuncts
            .iter()
            .enumerate()
            .any(|(j, c)| j != index && (c.support.contains(var) || c.owner == Some(*var)))
    };

    conjuncts[index]
        .support
        .iter()
        .filter(|var| smoothing.contains(var) && !mentioned_elsewhere(var))
        .count()
}

#[cfg(test)]
mod tests {
    use sas_plus::{Fact, Operator, State, Task, Variable};

    use super::*;
    use crate::VariableOrder;

    /// A conjunct over `support` owned by `owner`, without a diagram.
    fn conjunct(owner: VarId, support: &[VarId]) -> Conjunct<()> {
        Conjunct {
            diagram: (),
            support: support.iter().copied().collect(),
            owner: Some(owner),
        }
    }

    /// The owners of `conjuncts` in order.
    fn owners(conjuncts: &[Conjunct<()>]) -> Vec<VarId> {
        conjuncts.iter().filter_map(|c| c.owner).collect()
    }

    /// An encoding of `n` binary variables in task order: the current bit of
    /// variable `i` sits at level `2i`.
    fn binary_encoding(n: usize) -> Encoding {
        let task = Task {
            variables: (0..n).map(|i| Variable::new(format!("v{i}"), 2)).collect(),
            operators: Vec::<Operator>::new(),
            initial: State(vec![0; n]),
            goal: Vec::<Fact>::new(),
            axioms: vec![],
        };

        Encoding::new(&task, VariableOrder::Task).unwrap()
    }

    /// Hand-checked: with `C_0 = {0, 1}`, `C_1 = {1}`, `C_2 = {2, 0}`, only
    /// `C_2` retires a variable at first (2). Its IWLS95 benefit is
    /// `2·½ + ⅔ + ⅓ + 4/4 = 3`, against `1.5` and `7/6`. Then `C_0` retires 0.
    #[test]
    fn both_schedules_on_a_hand_checked_instance() {
        let encoding = binary_encoding(3);
        let make = || {
            vec![
                conjunct(0, &[0, 1]),
                conjunct(1, &[1]),
                conjunct(2, &[2, 0]),
            ]
        };
        let smoothing = [0, 1, 2].into_iter().collect();

        let benefit = |i| iwls95_benefit(&encoding, &make(), i, &smoothing);
        assert!((benefit(2) - 3.0).abs() < 1e-9);
        assert!((benefit(0) - 1.5).abs() < 1e-9);
        assert!((benefit(1) - 7.0 / 6.0).abs() < 1e-9);

        for schedule in [Schedule::Greedy, Schedule::Iwls95] {
            assert_eq!(owners(&order(&encoding, make(), schedule)), [2, 0, 1]);
        }
    }

    /// The level term separates conjuncts the greedy order ties: both
    /// retire their own variable, greedy takes the lower id, IWLS95 the
    /// deeper one.
    #[test]
    fn iwls95_prefers_deeper_variables_on_greedy_ties() {
        let encoding = binary_encoding(2);
        let make = || vec![conjunct(0, &[0]), conjunct(1, &[1])];

        assert_eq!(owners(&order(&encoding, make(), Schedule::Greedy)), [0, 1]);
        assert_eq!(owners(&order(&encoding, make(), Schedule::Iwls95)), [1, 0]);
    }
}
