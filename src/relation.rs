//! Transition relations, image and preimage, and their disjunctive
//! partitioning.
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
//!
//! **Merged groups.** One image per operator is many small relational
//! products. [`partition`] merges operators of equal cost into disjunctive
//! groups `T_G = ⋁_{o ∈ G} T_o ∧ ⋀_{v ∈ eff(G) ∖ eff(o)} v' = v`, so the
//! frame axioms cover the variables only some members touch, and the
//! group quantifies and renames `eff(G)`. A group grows while its BDD stays
//! within a node limit. Per-operator relations stay around for plan
//! reconstruction.
//!
//! **Conjunctive clusters.** Alternatively, [`conjunctive`] splits each
//! operator's relation into per-variable conjuncts, clustered up to a node
//! limit, and computes images and preimages by early quantification. See
//! that module.
//!
//! **Projection.** [`Relation::project`] forgets dropped variables, which
//! gives the exact transition relation of the domain abstraction that
//! drops them. Projected relations are always monolithic: `∃` does not
//! distribute over `∧`, so clusters are not projected one by one.

pub(crate) mod conjunctive;

use std::collections::BTreeMap;

use sas_plus::{Effect, Operator, VarId};

use crate::backend::{Backend, BackendResult};
use crate::encoding::{Encoding, Frame};
use crate::model::Direction;

/// A transition relation over a set of affected variables, with prebuilt
/// quantification sets and renamings. Either one operator or a merged group
/// of operators that share a cost.
pub(crate) struct Relation<B: Backend> {
    /// Cost of every transition in this relation.
    pub cost: u64,
    /// The affected variables, sorted and without duplicates.
    vars: Vec<VarId>,
    /// `T(x, x')`, over current bits of all mentioned variables and next
    /// bits of the affected ones.
    diagram: B::Set,
    /// Current bits of the affected variables (quantified by the image).
    current: B::VarSet,
    /// Next bits of the affected variables (quantified by the preimage).
    next: B::VarSet,
    /// Next bits to current bits of the affected variables.
    to_current: B::Renaming,
    /// Current bits to next bits of the affected variables.
    to_next: B::Renaming,
    /// Conjunctive clusters in schedule order; empty for a monolithic
    /// relation, which steps with `diagram` in one relational product.
    clusters: Vec<conjunctive::Cluster<B>>,
}

impl<B: Backend> Relation<B> {
    /// Builds the relation of `operator`.
    ///
    /// # Errors
    ///
    /// Out of memory in the backend.
    pub(crate) fn new(b: &B, encoding: &Encoding, operator: &Operator) -> BackendResult<Self> {
        let vars = affected_vars(operator);

        let mut relation = encoding.facts(b, &operator.pre)?;
        for &var in &vars {
            relation = b.and(&relation, &encoding.valid(b, var)?)?;
            relation = b.and(&relation, &next_value(b, encoding, var, &operator.eff)?)?;
        }

        Self::over(b, encoding, u64::from(operator.cost), vars, relation)
    }

    /// Wraps `diagram` on the affected variables `vars` with its
    /// quantification sets and renamings.
    ///
    /// # Errors
    ///
    /// Out of memory in the backend.
    fn over(
        b: &B,
        encoding: &Encoding,
        cost: u64,
        vars: Vec<VarId>,
        diagram: B::Set,
    ) -> BackendResult<Self> {
        let bits = |frame| -> Vec<u32> {
            vars.iter()
                .flat_map(|&var| encoding.diagram_vars(var, frame))
                .collect()
        };
        let (current, next) = (bits(Frame::Current), bits(Frame::Next));

        Ok(Self {
            cost,
            diagram,
            current: b.var_set(&current)?,
            next: b.var_set(&next)?,
            to_current: b.renaming(&next, &current),
            to_next: b.renaming(&current, &next),
            vars,
            clusters: Vec::new(),
        })
    }

    /// The disjunction of `self` and `other`, with frame axioms for the
    /// variables only one of them affects. Both must have the same cost.
    ///
    /// # Errors
    ///
    /// Out of memory in the backend.
    fn merge(&self, b: &B, encoding: &Encoding, other: &Self) -> BackendResult<Self> {
        debug_assert_eq!(self.cost, other.cost, "groups never mix costs");
        let mut vars: Vec<VarId> = self.vars.iter().chain(&other.vars).copied().collect();
        vars.sort_unstable();
        vars.dedup();

        let framed = |part: &Self| -> BackendResult<B::Set> {
            vars.iter()
                .filter(|var| part.vars.binary_search(var).is_err())
                .try_fold(part.diagram.clone(), |acc, &var| {
                    b.and(&acc, &encoding.unchanged(b, var)?)
                })
        };
        let relation = b.or(&framed(self)?, &framed(other)?)?;

        Self::over(b, encoding, self.cost, vars, relation)
    }

    /// The abstract relation that forgets the variables in `dropped`: a
    /// transition exists between two abstract states when it exists between
    /// some valid concrete states they stand for.
    ///
    /// `valid_dropped` are the valid codes of the dropped variables; without
    /// them, an invalid code would disable every conditional effect on it and
    /// add spurious transitions. `dropped` holds the current and next bits of
    /// the dropped variables.
    ///
    /// # Errors
    ///
    /// Out of memory in the backend.
    pub(crate) fn project(
        &self,
        b: &B,
        encoding: &Encoding,
        valid_dropped: &B::Set,
        dropped: &B::VarSet,
    ) -> BackendResult<Self> {
        let relation = b.exists(&b.and(&self.diagram, valid_dropped)?, dropped)?;

        Self::over(b, encoding, self.cost, self.vars.clone(), relation)
    }

    /// A copy of `self` with its own quantification sets and renamings.
    ///
    /// # Errors
    ///
    /// Out of memory in the backend.
    fn duplicate(&self, b: &B, encoding: &Encoding) -> BackendResult<Self> {
        Self::over(
            b,
            encoding,
            self.cost,
            self.vars.clone(),
            self.diagram.clone(),
        )
    }

    /// Number of nodes of the relation BDD.
    fn size(&self, b: &B) -> usize {
        b.node_count(&self.diagram)
    }

    /// One step from `states`: [`Relation::image`] forward,
    /// [`Relation::preimage`] backward.
    ///
    /// # Errors
    ///
    /// Out of memory in the backend.
    pub(crate) fn step(
        &self,
        b: &B,
        states: &B::Set,
        direction: Direction,
    ) -> BackendResult<B::Set> {
        match direction {
            Direction::Forward => self.image(b, states),
            Direction::Backward => self.preimage(b, states),
        }
    }

    /// The states reached from `states` by one transition.
    ///
    /// # Errors
    ///
    /// Out of memory in the backend.
    pub(crate) fn image(&self, b: &B, states: &B::Set) -> BackendResult<B::Set> {
        if !self.clusters.is_empty() {
            return self.clustered_image(b, states);
        }

        let product = b.and_exists(states, &self.diagram, &self.current)?;

        b.rename(&product, &self.to_current)
    }

    /// The states from which one transition reaches `states`.
    ///
    /// # Errors
    ///
    /// Out of memory in the backend.
    pub(crate) fn preimage(&self, b: &B, states: &B::Set) -> BackendResult<B::Set> {
        if !self.clusters.is_empty() {
            return self.clustered_preimage(b, states);
        }

        let shifted = b.rename(states, &self.to_next)?;

        b.and_exists(&shifted, &self.diagram, &self.next)
    }
}

/// One conjunctively clustered relation per operator of `task`, sorted by
/// cost (stable, so operator order within a cost).
///
/// # Errors
///
/// Out of memory in the backend.
pub(crate) fn conjunctive_groups<B: Backend>(
    b: &B,
    encoding: &Encoding,
    operators: &[Operator],
    cluster_node_limit: usize,
) -> BackendResult<Vec<Relation<B>>> {
    let mut groups = operators
        .iter()
        .map(|operator| Relation::conjunctive(b, encoding, operator, cluster_node_limit))
        .collect::<BackendResult<Vec<_>>>()?;
    groups.sort_by_key(|group| group.cost);

    Ok(groups)
}

/// Merges `operators` into disjunctive groups of equal cost.
///
/// Within each cost, in operator order, a group absorbs the next operator
/// while the merged BDD has at most `node_limit` nodes. `node_limit` 0 keeps
/// one group per operator. Groups come out sorted by cost.
///
/// # Errors
///
/// Out of memory in the backend.
pub(crate) fn partition<B: Backend>(
    b: &B,
    encoding: &Encoding,
    operators: &[Relation<B>],
    node_limit: usize,
) -> BackendResult<Vec<Relation<B>>> {
    let mut by_cost: BTreeMap<u64, Vec<&Relation<B>>> = BTreeMap::new();
    for relation in operators {
        by_cost.entry(relation.cost).or_default().push(relation);
    }

    let mut groups = Vec::new();
    for members in by_cost.into_values() {
        merge_greedily(b, encoding, &members, node_limit, &mut groups)?;
    }

    Ok(groups)
}

/// Appends the greedy merge of `members` (all of one cost) to `groups`.
///
/// # Errors
///
/// Out of memory in the backend.
fn merge_greedily<B: Backend>(
    b: &B,
    encoding: &Encoding,
    members: &[&Relation<B>],
    node_limit: usize,
    groups: &mut Vec<Relation<B>>,
) -> BackendResult<()> {
    let mut current: Option<Relation<B>> = None;
    for &member in members {
        let next = match current.take() {
            None => member.duplicate(b, encoding)?,
            Some(group) => {
                let merged = group.merge(b, encoding, member)?;
                if merged.size(b) <= node_limit {
                    merged
                } else {
                    groups.push(group);
                    member.duplicate(b, encoding)?
                }
            }
        };
        current = Some(next);
    }

    groups.extend(current);

    Ok(())
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
