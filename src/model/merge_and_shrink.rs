//! Symbolic merge-and-shrink abstractions with a linear merge strategy and
//! an h-preserving, node-limited shrink step.
//!
//! An abstraction is a list of disjoint *classes*: BDDs over the bits of the
//! variables merged so far, each class one abstract state. Its transitions
//! are those of the projection onto the merged variables: class `C` reaches
//! class `D` at cost `c` when some state of `C` has an operator of cost `c`
//! into `D` there. Abstract distances are computed by a Dijkstra over
//! classes that steps whole buckets of classes at once.
//!
//! ```mermaid
//! graph LR
//!     Start[one class: all states] --> Merge[merge the next variable: split every class by its values]
//!     Merge --> Dist[class distances in the projection]
//!     Dist --> Big{classes over the node limit?}
//!     Big -- yes --> Shrink[merge classes of equal h, then of the smallest h]
//!     Big -- no --> More{variables left?}
//!     Shrink --> More
//!     More -- yes --> Merge
//!     More -- no --> Final[distances of the final classes as a partition]
//! ```
//!
//! **Merge strategy.** Linear: goal variables first, then the others, each
//! group by ascending id (the reverse of the `SymBA*` drop order).
//!
//! **Shrink.** When the classes need more than the node limit (sum of
//! their BDD sizes), classes with equal abstract distance are merged
//! (h-preserving: no abstract distance changes for the merged states'
//! classes at that point). If that is still too big, the two classes with
//! the smallest distances are merged until the classes fit or one is left.
//! Dead classes are only merged with dead classes by the first rule.
//!
//! **Admissibility.** Every class list is the quotient of a projection by
//! a partition of its states, so it is an abstraction of the task, and the
//! final distances are exact in it: admissible and consistent. With an
//! unlimited node limit and all variables merged, the classes are single
//! states and the heuristic is `h*` (a unit test pins both).

use std::collections::BTreeMap;

use sas_plus::{Fact, VarId};

use crate::backend::{Backend, BackendResult};
use crate::encoding::Frame;
use crate::model::heuristic::Partition;
use crate::model::space::Space;
use crate::model::{Direction, Model};

impl<B: Backend> Model<B> {
    /// The merge-and-shrink goal-distance heuristic with shrink limit
    /// `node_limit`, for forward search.
    ///
    /// # Errors
    ///
    /// Out of memory in the backend.
    pub(crate) fn merge_and_shrink(&self, node_limit: usize) -> BackendResult<Partition<B>> {
        self.merge_and_shrink_towards(Direction::Backward, node_limit)
    }

    /// The merge-and-shrink distances from the start of `direction` (goal
    /// distances for [`Direction::Backward`], initial-state distances for
    /// [`Direction::Forward`]), as a heuristic for the opposite direction.
    ///
    /// # Errors
    ///
    /// Out of memory in the backend.
    pub(crate) fn merge_and_shrink_towards(
        &self,
        direction: Direction,
        node_limit: usize,
    ) -> BackendResult<Partition<B>> {
        let b = &self.backend;
        let order = self.merge_order();
        let mut classes = vec![b.constant(true)];
        for (merged, &var) in order.iter().enumerate() {
            classes = self.split(&classes, var)?;
            if total_nodes(b, &classes) <= node_limit {
                continue;
            }

            let space = self.project(&order[merged + 1..])?;
            let distances = class_distances(b, &space, &classes, direction)?;
            classes = shrink(b, classes, &distances, node_limit)?;
        }

        let distances = class_distances(b, &self.concrete, &classes, direction)?;
        let pairs = classes
            .into_iter()
            .zip(distances)
            .filter_map(|(class, d)| Some((d?, class)))
            .collect();

        Partition::from_overlapping(b, pairs, None)
    }

    /// The linear merge order: goal variables, then the others, each by
    /// ascending id, without zero-bit variables (which never split a class).
    fn merge_order(&self) -> Vec<VarId> {
        let is_goal = |var: &VarId| self.goal_vars.binary_search(var).is_ok();
        let has_bits = |var: &VarId| self.encoding.diagram_vars(*var, Frame::Current).count() > 0;
        let all = 0..self.encoding.num_vars();

        all.clone()
            .filter(is_goal)
            .chain(all.filter(|var| !is_goal(var)))
            .filter(has_bits)
            .collect()
    }

    /// The merge step: every class split by the values of `var`, empty parts
    /// dropped.
    ///
    /// # Errors
    ///
    /// Out of memory in the backend.
    fn split(&self, classes: &[B::Set], var: VarId) -> BackendResult<Vec<B::Set>> {
        let b = &self.backend;
        let values = (0..self.encoding.domain(var))
            .map(|value| {
                self.encoding
                    .value(b, Fact::new(var, value), Frame::Current)
            })
            .collect::<BackendResult<Vec<_>>>()?;

        let mut split = Vec::with_capacity(classes.len() * values.len());
        for class in classes {
            for value in &values {
                let part = b.and(class, value)?;
                if !b.is_empty(&part) {
                    split.push(part);
                }
            }
        }

        Ok(split)
    }
}

/// Sum of the BDD sizes of `classes`.
fn total_nodes<B: Backend>(b: &B, classes: &[B::Set]) -> usize {
    classes.iter().map(|class| b.node_count(class)).sum()
}

/// The abstract distance of every class from the start of `direction` in
/// `space`, `None` when unreachable: a Dijkstra over classes that settles
/// all classes of the cheapest bucket together and steps their union.
///
/// # Errors
///
/// Out of memory in the backend.
pub(crate) fn class_distances<B: Backend>(
    b: &B,
    space: &Space<B>,
    classes: &[B::Set],
    direction: Direction,
) -> BackendResult<Vec<Option<u64>>> {
    let mut distances: Vec<Option<u64>> = vec![None; classes.len()];
    let mut open: BTreeMap<u64, Vec<usize>> = BTreeMap::new();
    let start = space.start(direction);
    for (i, class) in classes.iter().enumerate() {
        if !b.is_empty(&b.and(class, start)?) {
            open.entry(0).or_default().push(i);
        }
    }

    while let Some((g, bucket)) = open.pop_first() {
        let fresh: Vec<usize> = bucket
            .into_iter()
            .filter(|&i| distances[i].is_none())
            .collect();
        if fresh.is_empty() {
            continue;
        }

        let mut union = b.constant(false);
        for &i in &fresh {
            distances[i] = Some(g);
            union = b.or(&union, &classes[i])?;
        }

        for group in &space.groups {
            let next = group.step(b, &union, direction)?;
            if b.is_empty(&next) {
                continue;
            }

            for (j, class) in classes.iter().enumerate() {
                if distances[j].is_none() && !b.is_empty(&b.and(class, &next)?) {
                    open.entry(g.saturating_add(group.cost))
                        .or_default()
                        .push(j);
                }
            }
        }
    }

    Ok(distances)
}

/// The shrink step (see the module docs): classes of equal distance merged,
/// then the two with the smallest distances while the classes need more
/// than `node_limit` nodes.
///
/// # Errors
///
/// Out of memory in the backend.
fn shrink<B: Backend>(
    b: &B,
    classes: Vec<B::Set>,
    distances: &[Option<u64>],
    node_limit: usize,
) -> BackendResult<Vec<B::Set>> {
    let mut by_h: BTreeMap<u64, B::Set> = BTreeMap::new();
    for (class, d) in classes.into_iter().zip(distances) {
        // Dead classes sort last and only join each other.
        let key = d.unwrap_or(u64::MAX);
        let merged = match by_h.remove(&key) {
            Some(old) => b.or(&old, &class)?,
            None => class,
        };
        by_h.insert(key, merged);
    }

    while by_h.len() > 1 && by_h.values().map(|c| b.node_count(c)).sum::<usize>() > node_limit {
        let (_, first) = by_h.pop_first().expect("two classes");
        let (key, second) = by_h.pop_first().expect("two classes");
        by_h.insert(key, b.or(&first, &second)?);
    }

    Ok(by_h.into_values().collect())
}
