//! Domain abstractions that drop variables, and the heuristics built from
//! them: symbolic pattern databases and `SymBA*` perimeters.
//!
//! Dropping a variable means quantifying its bits out of every relation and
//! of the initial and goal sets. Abstract sets then never mention the
//! dropped bits, so an abstract set read as a concrete set contains all
//! concrete states that map to it. A blind Dijkstra in the abstract space
//! gives abstract distances, which are admissible and consistent for the
//! concrete space.

use sas_plus::VarId;

use crate::backend::{Backend, BackendResult};
use crate::encoding::Frame;
use crate::model::frontier::{Frontier, Strategy};
use crate::model::heuristic::Partition;
use crate::model::space::Space;
use crate::model::{Direction, Model};

impl<B: Backend> Model<B> {
    /// The concrete space with the variables in `dropped` forgotten.
    ///
    /// # Errors
    ///
    /// Out of memory in the backend.
    pub(crate) fn project(&self, dropped: &[VarId]) -> BackendResult<Space<B>> {
        let (b, encoding) = (&self.backend, &self.encoding);
        let bits: Vec<u32> = dropped
            .iter()
            .flat_map(|&var| {
                encoding
                    .diagram_vars(var, Frame::Current)
                    .chain(encoding.diagram_vars(var, Frame::Next))
            })
            .collect();
        let quantified = b.var_set(&bits)?;
        let valid = dropped.iter().try_fold(b.constant(true), |acc, &var| {
            b.and(&acc, &encoding.valid(b, var)?)
        })?;

        let groups = self
            .concrete
            .groups
            .iter()
            .map(|group| group.project(b, encoding, &valid, &quantified))
            .collect::<BackendResult<Vec<_>>>()?;

        Ok(Space {
            groups,
            initial: b.exists(&self.concrete.initial, &quantified)?,
            goal: b.exists(&self.concrete.goal, &quantified)?,
        })
    }

    /// The symbolic pattern database of `pattern`: exact goal distances in
    /// the projection onto `pattern`, states that cannot reach the abstract
    /// goal marked dead. `pattern` must hold valid variable ids.
    ///
    /// # Errors
    ///
    /// Out of memory in the backend.
    pub(crate) fn pdb(&self, pattern: &[VarId]) -> BackendResult<Partition<B>> {
        let dropped: Vec<VarId> = (0..self.encoding.num_vars())
            .filter(|var| !pattern.contains(var))
            .collect();
        let space = self.project(&dropped)?;

        self.perimeter(&space, Direction::Backward, usize::MAX)
    }

    /// Runs a blind Dijkstra in `space` in `direction` until it is exhausted
    /// or, after the first expansion, its next bucket has more than
    /// `node_limit` nodes, and turns the
    /// closed layers into a heuristic for the opposite direction.
    ///
    /// Closed states get their abstract distance. When the search stopped
    /// early, all other states get the smallest open cost (the perimeter);
    /// when it ran out, they are dead ends.
    ///
    /// # Errors
    ///
    /// Out of memory in the backend.
    pub(crate) fn perimeter(
        &self,
        space: &Space<B>,
        direction: Direction,
        node_limit: usize,
    ) -> BackendResult<Partition<B>> {
        let b = &self.backend;
        let mut frontier =
            Frontier::new(b, space, direction, Partition::zero(b), Strategy::Cheapest)?;
        // The first bucket is always expanded, so even a tiny limit gives an
        // informed perimeter.
        while let Some((g, states)) = frontier.pop()? {
            frontier.expand(g, &states)?;
            if frontier.next_size() > node_limit {
                break;
            }
        }

        perimeter_of(b, &frontier)
    }
}

/// The heuristic for the opposite direction that the blind abstract
/// `frontier` gives at this point: closed states get their abstract
/// distance, all other states the smallest open cost, or are dead ends when
/// open is empty. Admissible and consistent at every pause point.
///
/// # Errors
///
/// Out of memory in the backend.
pub(crate) fn perimeter_of<B: Backend>(
    b: &B,
    frontier: &Frontier<'_, B>,
) -> BackendResult<Partition<B>> {
    let pairs = frontier
        .closed()
        .iter()
        .map(|(&g, set)| (g, set.clone()))
        .collect();

    Partition::from_overlapping(b, pairs, frontier.min_g())
}
