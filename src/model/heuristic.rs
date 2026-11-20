//! Heuristics as a partition of the state space into BDDs by `h` value.
//!
//! `BDDA*` and `SymBA*` evaluate a heuristic on a whole set at once by
//! intersecting it with one BDD per `h` value (Torralba et al. 2017). The
//! partition covers the states the heuristic knows to be alive; the
//! complement holds the proven dead ends, which the search prunes.

use std::collections::BTreeMap;

use crate::backend::{Backend, BackendResult};

/// A heuristic as disjoint sets of states, one per `h` value.
pub(crate) struct Partition<B: Backend> {
    /// `(h, states)` pairs, sorted by ascending `h`, with pairwise disjoint
    /// and non-empty sets. States outside every set are dead ends.
    buckets: Vec<(u64, B::Set)>,
}

impl<B: Backend> Clone for Partition<B> {
    fn clone(&self) -> Self {
        Self {
            buckets: self.buckets.clone(),
        }
    }
}

impl<B: Backend> Partition<B> {
    /// The blind heuristic: `h = 0` everywhere, no dead ends.
    pub(crate) fn zero(b: &B) -> Self {
        Self {
            buckets: vec![(0, b.constant(true))],
        }
    }

    /// Builds a partition from possibly overlapping `(h, set)` pairs, giving
    /// each state the largest `h` of the pairs that contain it. States in no
    /// pair get `rest`, or are dead ends when `rest` is `None`.
    ///
    /// # Errors
    ///
    /// Out of memory in the backend.
    pub(crate) fn from_overlapping(
        b: &B,
        mut pairs: Vec<(u64, B::Set)>,
        rest: Option<u64>,
    ) -> BackendResult<Self> {
        pairs.sort_by_key(|&(h, _)| core::cmp::Reverse(h));

        let mut by_h: BTreeMap<u64, B::Set> = BTreeMap::new();
        let mut covered = b.constant(false);
        for (h, set) in pairs {
            let fresh = b.and(&set, &b.not(&covered)?)?;
            covered = b.or(&covered, &fresh)?;
            add(b, &mut by_h, h, &fresh)?;
        }

        if let Some(h) = rest {
            add(b, &mut by_h, h, &b.not(&covered)?)?;
        }

        Ok(Self::from_map(b, by_h))
    }

    /// The pointwise maximum of `self` and `other`. A state is dead when
    /// either side says so. The maximum of two admissible (consistent)
    /// heuristics is admissible (consistent).
    ///
    /// # Errors
    ///
    /// Out of memory in the backend.
    pub(crate) fn max(&self, b: &B, other: &Self) -> BackendResult<Self> {
        self.combine(b, other, u64::max)
    }

    /// The pointwise sum of `self` and `other` (saturating). A state is dead
    /// when either side says so. Admissible when the two heuristics are
    /// additive, for example PDBs of patterns no operator affects both of.
    ///
    /// # Errors
    ///
    /// Out of memory in the backend.
    pub(crate) fn sum(&self, b: &B, other: &Self) -> BackendResult<Self> {
        self.combine(b, other, u64::saturating_add)
    }

    /// The pointwise combination of `self` and `other` by `op`, dead where
    /// either is dead.
    ///
    /// # Errors
    ///
    /// Out of memory in the backend.
    fn combine(&self, b: &B, other: &Self, op: fn(u64, u64) -> u64) -> BackendResult<Self> {
        let mut by_h: BTreeMap<u64, B::Set> = BTreeMap::new();
        for (h1, s1) in &self.buckets {
            for (h2, s2) in &other.buckets {
                add(b, &mut by_h, op(*h1, *h2), &b.and(s1, s2)?)?;
            }
        }

        Ok(Self::from_map(b, by_h))
    }

    /// Drops empty sets from `by_h` and keeps the rest in ascending order.
    fn from_map(b: &B, by_h: BTreeMap<u64, B::Set>) -> Self {
        Self {
            buckets: by_h
                .into_iter()
                .filter(|(_, set)| !b.is_empty(set))
                .collect(),
        }
    }

    /// Whether this is the blind heuristic: `h = 0` everywhere.
    pub(crate) fn is_blind(&self, b: &B) -> bool {
        matches!(self.buckets.as_slice(), [(0, all)] if *all == b.constant(true))
    }

    /// The `(h, states)` pairs in ascending `h`.
    pub(crate) fn buckets(&self) -> &[(u64, B::Set)] {
        &self.buckets
    }

    /// The `h` value of the states in `set`, taken from the first bucket
    /// `set` meets, or `None` when every state of `set` is a dead end. Exact
    /// for a singleton `set`.
    ///
    /// # Errors
    ///
    /// Out of memory in the backend.
    pub(crate) fn value(&self, b: &B, set: &B::Set) -> BackendResult<Option<u64>> {
        for (h, bucket) in &self.buckets {
            if !b.is_empty(&b.and(set, bucket)?) {
                return Ok(Some(*h));
            }
        }

        Ok(None)
    }
}

/// Adds `set` to the bucket of `h` in `by_h`.
///
/// # Errors
///
/// Out of memory in the backend.
fn add<B: Backend>(
    b: &B,
    by_h: &mut BTreeMap<u64, B::Set>,
    h: u64,
    set: &B::Set,
) -> BackendResult<()> {
    let merged = match by_h.get(&h) {
        Some(old) => b.or(old, set)?,
        None => set.clone(),
    };
    by_h.insert(h, merged);

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::backend::Capacity;
    use crate::backend::bcdd::Oxidd;

    /// A two-variable manager for partition tests.
    fn manager() -> Oxidd {
        Oxidd::new(
            2,
            Capacity {
                nodes: 256,
                cache: 256,
            },
        )
        .unwrap()
    }

    /// Overlaps resolve to the larger `h`; the rest gets the default.
    #[test]
    fn overlapping_pairs_keep_the_maximum() {
        let b = manager();
        let (x, y) = (b.var(0), b.var(1));
        let both = b.and(&x, &y).unwrap();

        let p =
            Partition::from_overlapping(&b, vec![(1, x.clone()), (3, y.clone())], Some(0)).unwrap();

        assert_eq!(p.value(&b, &both).unwrap(), Some(3));
        assert_eq!(
            p.value(&b, &b.and(&x, &b.not(&y).unwrap()).unwrap())
                .unwrap(),
            Some(1)
        );
        let neither = b.and(&b.not(&x).unwrap(), &b.not(&y).unwrap()).unwrap();
        assert_eq!(p.value(&b, &neither).unwrap(), Some(0));
    }

    /// The maximum takes the larger value and keeps dead ends of either side.
    #[test]
    fn max_combines_values_and_dead_ends() {
        let b = manager();
        let x = b.var(0);
        let not_x = b.not(&x).unwrap();
        let left = Partition::from_overlapping(&b, vec![(2, x.clone()), (5, not_x.clone())], None)
            .unwrap();
        let right = Partition::from_overlapping(&b, vec![(4, x.clone())], None).unwrap();

        let max = left.max(&b, &right).unwrap();

        assert_eq!(max.value(&b, &x).unwrap(), Some(4));
        assert_eq!(max.value(&b, &not_x).unwrap(), None);
    }

    /// The sum adds values and keeps dead ends of either side.
    #[test]
    fn sum_adds_values_and_keeps_dead_ends() {
        let b = manager();
        let x = b.var(0);
        let not_x = b.not(&x).unwrap();
        let left = Partition::from_overlapping(&b, vec![(2, x.clone()), (5, not_x.clone())], None)
            .unwrap();
        let right = Partition::from_overlapping(&b, vec![(4, x.clone())], None).unwrap();

        let sum = left.sum(&b, &right).unwrap();

        assert_eq!(sum.value(&b, &x).unwrap(), Some(6));
        assert_eq!(sum.value(&b, &not_x).unwrap(), None);
    }
}
