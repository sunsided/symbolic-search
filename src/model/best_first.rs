//! Unidirectional cost-aware search: symbolic Dijkstra, `BDDA*` and GBFS.
//!
//! ```text
//! open = {(0, start)}
//! loop:
//!     (g, S) = pop the best bucket, minus states closed by g
//!     if S ∧ target ≠ ⊥: trace one state of it back, done
//!     close S at g; file each group's step from S at g + cost
//! ```
//!
//! With [`Strategy::Cheapest`] and an admissible heuristic the first hit has
//! optimal cost: every bucket that could lead to a cheaper goal has a
//! smaller `g + h` and is popped first.

use crate::backend::Backend;
use crate::model::frontier::{Frontier, Strategy};
use crate::model::heuristic::Partition;
use crate::model::{Direction, Model, orient};
use crate::{Error, Solution};

impl<B: Backend> Model<B> {
    /// Best-first search in `direction` with `heuristic` and `strategy`.
    ///
    /// Returns a plan in execution order, or `None` when the target is not
    /// reachable. Optimal for [`Strategy::Cheapest`] with an admissible
    /// heuristic.
    ///
    /// # Errors
    ///
    /// [`Error::OutOfMemory`] when the node table fills up.
    pub(crate) fn best_first(
        &self,
        direction: Direction,
        heuristic: Partition<B>,
        strategy: Strategy,
    ) -> Result<Option<Solution>, Error> {
        let b = &self.backend;
        let target = self.concrete.start(direction.reverse());
        let mut frontier = Frontier::new(b, &self.concrete, direction, heuristic, strategy)?;
        while let Some((g, states)) = frontier.pop()? {
            let hit = b.and(&states, target)?;
            if !b.is_empty(&hit) {
                let state = self.pick_state(&hit)?;
                let plan = self.trace_frontier(&frontier, state, g)?;
                return Ok(Some(self.solution(orient(plan, direction))));
            }

            frontier.expand(g, &states)?;
        }

        Ok(None)
    }
}
