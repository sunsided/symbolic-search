//! Conjunctive partitioning of one operator's relation, with early
//! quantification.
//!
//! The relation of an operator `o` is already a conjunction, one conjunct per
//! affected variable plus one for the remaining preconditions:
//!
//! ```text
//! T_o = pre_rest(x) ∧ ⋀_{v ∈ eff(o)} C_v,   C_v = pre_v(x) ∧ valid_v(x) ∧ E_v(x, x')
//! ```
//!
//! `pre_v` are the precondition facts on `v` and `pre_rest` those on
//! untouched variables. Untouched variables keep their frame for free: they
//! are never quantified or renamed, exactly as in the monolithic relation.
//!
//! **Schedule.** `pre_rest` goes first (it only filters). The other
//! conjuncts are ordered by [`Schedule::Greedy`] (most retired variables) or
//! by the [`Schedule::Iwls95`] benefit function; see the `schedule`
//! submodule. Consecutive conjuncts are then merged into clusters while a
//! cluster's BDD has at most the cluster node limit.
//!
//! **Image.** The current bits of `v` appear in `C_v` and in every conjunct
//! whose effect conditions or preconditions name `v`. They are quantified in
//! the last cluster that mentions `v`:
//!
//! ```text
//! image(S) = (∃ q_k. (… (∃ q_1. S ∧ K_1) …) ∧ K_k)[x'_eff ↦ x_eff]
//! ```
//!
//! **Preimage.** The next bits of `v` appear only in `C_v`, so they are
//! quantified in `C_v`'s cluster. Both results equal the monolithic image and
//! preimage exactly; a unit test pins that.
//!
//! This only splits one operator: a disjunction of operators does not
//! distribute over the conjuncts, so the conjunctive mode keeps one relation
//! per operator instead of merged groups.

mod schedule;

use std::collections::BTreeSet;

use sas_plus::{Operator, VarId};

use crate::Schedule;
use crate::backend::{Backend, BackendResult};
use crate::encoding::{Encoding, Frame};
use crate::relation::{Relation, affected_vars, next_value};

/// A cluster of conjuncts with the bits it quantifies early.
pub(crate) struct Cluster<B: Backend> {
    /// The conjunction of the cluster's conjuncts.
    diagram: B::Set,
    /// Current bits of the affected variables no later cluster mentions;
    /// quantified by the image right after this cluster.
    retire_current: B::VarSet,
    /// Next bits of the affected variables whose conjunct is in this cluster;
    /// quantified by the preimage right after this cluster.
    retire_next: B::VarSet,
}

/// One conjunct of an operator relation before clustering.
struct Conjunct<S> {
    /// The conjunct's BDD.
    diagram: S,
    /// SAS+ variables whose current bits the conjunct may mention. An
    /// over-approximation is safe; an under-approximation is not.
    support: BTreeSet<VarId>,
    /// The affected variable whose next bits this conjunct holds, `None` for
    /// the precondition conjunct on untouched variables.
    owner: Option<VarId>,
}

/// A run of consecutive scheduled conjuncts merged into one BDD.
struct Merged<S> {
    /// The conjunction.
    diagram: S,
    /// Union of the members' supports.
    support: BTreeSet<VarId>,
    /// The affected variables owned by the members.
    owners: Vec<VarId>,
}

impl<B: Backend> Relation<B> {
    /// Builds the relation of `operator`, split into scheduled clusters of
    /// at most `cluster_node_limit` nodes each (a single conjunct may be
    /// larger), in the order `schedule` picks. `0` keeps one cluster per
    /// conjunct.
    ///
    /// The monolithic diagram is kept for projection, merging and plan
    /// reconstruction; image and preimage use the clusters.
    ///
    /// # Errors
    ///
    /// Out of memory in the backend.
    pub(crate) fn conjunctive(
        b: &B,
        encoding: &Encoding,
        operator: &Operator,
        cluster_node_limit: usize,
        schedule: Schedule,
    ) -> BackendResult<Self> {
        let conjuncts = schedule::order(encoding, split(b, encoding, operator)?, schedule);
        let merged = cluster(b, conjuncts, cluster_node_limit)?;
        let clusters = quantification(b, encoding, &merged)?;

        let mut relation = Self::new(b, encoding, operator)?;
        relation.clusters = clusters;

        Ok(relation)
    }

    /// Number of clusters, 0 for a monolithic relation.
    #[cfg(test)]
    pub(crate) fn cluster_count(&self) -> usize {
        self.clusters.len()
    }

    /// The image of `states` with the largest intermediate product, in
    /// nodes, along the cluster chain: the measure a schedule minimises.
    ///
    /// # Errors
    ///
    /// Out of memory in the backend.
    #[cfg(test)]
    pub(crate) fn image_with_peak(&self, b: &B, states: &B::Set) -> BackendResult<(B::Set, usize)> {
        let mut peak = b.node_count(states);
        let mut product = states.clone();
        for c in &self.clusters {
            product = b.and_exists(&product, &c.diagram, &c.retire_current)?;
            peak = peak.max(b.node_count(&product));
        }

        Ok((b.rename(&product, &self.to_current)?, peak))
    }

    /// [`Relation::image`] by early quantification over the clusters.
    ///
    /// # Errors
    ///
    /// Out of memory in the backend.
    pub(super) fn clustered_image(&self, b: &B, states: &B::Set) -> BackendResult<B::Set> {
        let product = self.clusters.iter().try_fold(states.clone(), |acc, c| {
            b.and_exists(&acc, &c.diagram, &c.retire_current)
        })?;

        b.rename(&product, &self.to_current)
    }

    /// [`Relation::preimage`] by early quantification over the clusters.
    ///
    /// # Errors
    ///
    /// Out of memory in the backend.
    pub(super) fn clustered_preimage(&self, b: &B, states: &B::Set) -> BackendResult<B::Set> {
        let shifted = b.rename(states, &self.to_next)?;

        self.clusters.iter().try_fold(shifted, |acc, c| {
            b.and_exists(&acc, &c.diagram, &c.retire_next)
        })
    }
}

/// The conjuncts of `operator`: the preconditions on untouched variables
/// (left out when there are none), then one per affected variable.
///
/// # Errors
///
/// Out of memory in the backend.
fn split<B: Backend>(
    b: &B,
    encoding: &Encoding,
    operator: &Operator,
) -> BackendResult<Vec<Conjunct<B::Set>>> {
    let vars = affected_vars(operator);
    let is_affected = |var: &VarId| vars.binary_search(var).is_ok();

    let mut conjuncts = Vec::with_capacity(vars.len() + 1);
    let rest: Vec<_> = operator
        .pre
        .iter()
        .filter(|f| !is_affected(&f.var))
        .copied()
        .collect();
    if !rest.is_empty() {
        conjuncts.push(Conjunct {
            diagram: encoding.facts(b, &rest)?,
            support: rest.iter().map(|f| f.var).collect(),
            owner: None,
        });
    }

    for &var in &vars {
        conjuncts.push(variable_conjunct(b, encoding, operator, var)?);
    }

    Ok(conjuncts)
}

/// `C_v = pre_v ∧ valid_v ∧ E_v` for the affected variable `var`, with its
/// support: `var` and every variable named in a condition of an effect on
/// `var`.
///
/// # Errors
///
/// Out of memory in the backend.
fn variable_conjunct<B: Backend>(
    b: &B,
    encoding: &Encoding,
    operator: &Operator,
    var: VarId,
) -> BackendResult<Conjunct<B::Set>> {
    let pre: Vec<_> = operator
        .pre
        .iter()
        .filter(|f| f.var == var)
        .copied()
        .collect();
    let diagram = b.and(&encoding.facts(b, &pre)?, &encoding.valid(b, var)?)?;
    let diagram = b.and(&diagram, &next_value(b, encoding, var, &operator.eff)?)?;

    let support = operator
        .eff
        .iter()
        .filter(|e| e.fact.var == var)
        .flat_map(|e| e.conditions.iter().map(|c| c.var))
        .chain([var])
        .collect();

    Ok(Conjunct {
        diagram,
        support,
        owner: Some(var),
    })
}

/// Merges consecutive scheduled conjuncts while the merged BDD has at most
/// `node_limit` nodes.
///
/// # Errors
///
/// Out of memory in the backend.
fn cluster<B: Backend>(
    b: &B,
    conjuncts: Vec<Conjunct<B::Set>>,
    node_limit: usize,
) -> BackendResult<Vec<Merged<B::Set>>> {
    let mut clusters: Vec<Merged<B::Set>> = Vec::new();
    for conjunct in conjuncts {
        if let Some(last) = clusters.last_mut() {
            let diagram = b.and(&last.diagram, &conjunct.diagram)?;
            if b.node_count(&diagram) <= node_limit {
                last.diagram = diagram;
                last.support.extend(conjunct.support);
                last.owners.extend(conjunct.owner);
                continue;
            }
        }

        clusters.push(Merged {
            diagram: conjunct.diagram,
            support: conjunct.support,
            owners: conjunct.owner.into_iter().collect(),
        });
    }

    Ok(clusters)
}

/// Attaches the early quantification sets to `merged`: current bits of an
/// affected variable go to the last cluster that mentions it, next bits to
/// the cluster of its conjunct.
///
/// # Errors
///
/// Out of memory in the backend.
fn quantification<B: Backend>(
    b: &B,
    encoding: &Encoding,
    merged: &[Merged<B::Set>],
) -> BackendResult<Vec<Cluster<B>>> {
    let affected: BTreeSet<VarId> = merged.iter().flat_map(|m| m.owners.clone()).collect();
    let last_mention = |var: VarId| {
        merged
            .iter()
            .rposition(|m| m.support.contains(&var) || m.owners.contains(&var))
            .expect("every affected variable has its own conjunct")
    };
    let bits = |vars: &[VarId], frame| -> Vec<u32> {
        vars.iter()
            .flat_map(|&var| encoding.diagram_vars(var, frame))
            .collect()
    };

    let mut clusters = Vec::with_capacity(merged.len());
    for (index, m) in merged.iter().enumerate() {
        let retired: Vec<VarId> = affected
            .iter()
            .copied()
            .filter(|&var| last_mention(var) == index)
            .collect();
        clusters.push(Cluster {
            diagram: m.diagram.clone(),
            retire_current: b.var_set(&bits(&retired, Frame::Current))?,
            retire_next: b.var_set(&bits(&m.owners, Frame::Next))?,
        });
    }

    Ok(clusters)
}
