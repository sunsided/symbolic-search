//! Per-operator transition relations, image and preimage.
//!
//! The relation of operator `o` only mentions the variables `o` touches:
//!
//! ```text
//! T_o(x, x') = pre(x) ∧ ⋀_{v ∈ eff(o)} valid_v(x) ∧ E_v(x, x')
//! ```
//!
//! `E_v` says what the next value of `v` is. With the effects on `v` listed
//! in operator order as `(C₁ → a₁), …, (C_k → a_k)`, the last firing effect
//! wins, as in [`sas_plus::Task::apply`]:
//!
//! ```text
//! E_v = ite(C_k, v' = a_k, ite(C_{k-1}, v' = a_{k-1}, … ite(C₁, v' = a₁, v' = v)))
//! ```
//!
//! Variables outside `eff(o)` get no frame axiom. Instead, image and preimage
//! quantify and rename only the bits of `eff(o)`, so untouched bits pass
//! through unchanged:
//!
//! ```text
//! image_o(S)    = (∃ x_eff. S ∧ T_o)[x'_eff ↦ x_eff]
//! preimage_o(S) =  ∃ x'_eff. S[x_eff ↦ x'_eff] ∧ T_o
//! ```

use sas_plus::{Effect, OpId, Operator, VarId};

use crate::backend::{Backend, BackendResult};
use crate::encoding::{Encoding, Frame};

/// The transition relation of one operator, with its prebuilt quantification
/// sets and renamings.
pub(crate) struct OperatorRelation<B: Backend> {
    /// The operator this relation belongs to.
    pub op: OpId,
    /// `T_o(x, x')`, over current bits of all mentioned variables and next
    /// bits of the affected ones.
    relation: B::Set,
    /// Current bits of the affected variables (quantified by the image).
    current: B::VarSet,
    /// Next bits of the affected variables (quantified by the preimage).
    next: B::VarSet,
    /// Next bits to current bits of the affected variables.
    to_current: B::Renaming,
    /// Current bits to next bits of the affected variables.
    to_next: B::Renaming,
}

impl<B: Backend> OperatorRelation<B> {
    /// Builds the relation of operator `op`.
    ///
    /// # Errors
    ///
    /// Out of memory in the backend.
    pub(crate) fn new(
        b: &B,
        encoding: &Encoding,
        op: OpId,
        operator: &Operator,
    ) -> BackendResult<Self> {
        let affected = affected_vars(operator);

        let mut relation = encoding.facts(b, &operator.pre)?;
        for &var in &affected {
            relation = b.and(&relation, &encoding.valid(b, var)?)?;
            relation = b.and(&relation, &next_value(b, encoding, var, &operator.eff)?)?;
        }

        let bits = |frame| -> Vec<u32> {
            affected
                .iter()
                .flat_map(|&var| encoding.diagram_vars(var, frame))
                .collect()
        };
        let (current, next) = (bits(Frame::Current), bits(Frame::Next));

        Ok(Self {
            op,
            relation,
            current: b.var_set(&current)?,
            next: b.var_set(&next)?,
            to_current: b.renaming(&next, &current),
            to_next: b.renaming(&current, &next),
        })
    }

    /// The states reached from `states` by applying this operator once.
    ///
    /// # Errors
    ///
    /// Out of memory in the backend.
    pub(crate) fn image(&self, b: &B, states: &B::Set) -> BackendResult<B::Set> {
        let product = b.and_exists(states, &self.relation, &self.current)?;

        b.rename(&product, &self.to_current)
    }

    /// The states from which applying this operator once reaches `states`.
    ///
    /// # Errors
    ///
    /// Out of memory in the backend.
    pub(crate) fn preimage(&self, b: &B, states: &B::Set) -> BackendResult<B::Set> {
        let shifted = b.rename(states, &self.to_next)?;

        b.and_exists(&shifted, &self.relation, &self.next)
    }
}

/// The variables `operator` has an effect on, sorted and without duplicates.
fn affected_vars(operator: &Operator) -> Vec<VarId> {
    let mut vars: Vec<VarId> = operator.eff.iter().map(|e| e.fact.var).collect();
    vars.sort_unstable();
    vars.dedup();

    vars
}

/// `E_v`: the next value of `var` under the effects of one operator, folded
/// in operator order so that the last firing effect is checked first.
///
/// # Errors
///
/// Out of memory in the backend.
fn next_value<B: Backend>(
    b: &B,
    encoding: &Encoding,
    var: VarId,
    effects: &[Effect],
) -> BackendResult<B::Set> {
    let mut next = encoding.unchanged(b, var)?;
    for effect in effects.iter().filter(|e| e.fact.var == var) {
        let assigned = encoding.value(b, effect.fact, Frame::Next)?;
        let fires = encoding.facts(b, &effect.conditions)?;
        next = b.ite(&fires, &assigned, &next)?;
    }

    Ok(next)
}
