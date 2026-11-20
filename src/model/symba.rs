//! `SymBA*`: bidirectional symbolic A* with perimeter abstraction
//! heuristics (Torralba et al. 2014, 2016), in a simple form.
//!
//! ```mermaid
//! graph TD
//!     Pick[pick the direction with the smaller next bucket] --> Big{bucket over the node limit and an abstraction left?}
//!     Big -- no --> Expand[expand concretely, check meetings]
//!     Big -- yes --> Abstract[drop one more variable, run a bounded abstract search the other way]
//!     Abstract --> Max[h = max of old h and the perimeter heuristic, refile open]
//!     Max --> Pick
//!     Expand --> Pick
//! ```
//!
//! Both concrete frontiers start blind, so the search is bidirectional
//! Dijkstra until a frontier grows past the node limit. Then that direction
//! gets a heuristic from an abstract search in the opposite direction:
//! backward from the abstract goal for the forward frontier, forward from the
//! abstract initial state for the backward frontier. Abstraction `k` drops
//! `k` variables, non-goal variables first, so each new abstraction is
//! coarser and cheaper than the last. The abstract search expands at least
//! its start, then runs until its own next bucket passes the node limit; a
//! perimeter that comes out blind is skipped. The perimeter heuristic is
//! admissible
//! and consistent, and so is the maximum with the old heuristic, which keeps
//! the bidirectional termination bound valid.
//!
//! Simplifications against the full planner: abstractions are fixed variable
//! drops instead of merge-and-shrink or chosen patterns, the step choice uses
//! node counts instead of time estimates, and abstract searches are not
//! resumed.

use sas_plus::{OpId, VarId};

use crate::Error;
use crate::backend::Backend;
use crate::model::frontier::{Frontier, Strategy};
use crate::model::heuristic::Partition;
use crate::model::{Direction, Model};

impl<B: Backend> Model<B> {
    /// `SymBA*` with `node_limit` as the frontier size that triggers the next
    /// abstraction and bounds each abstract search.
    ///
    /// Returns an optimal plan in execution order, or `None` when the goal is
    /// unreachable, plus the number of informative (not blind) abstraction
    /// heuristics installed.
    ///
    /// # Errors
    ///
    /// [`Error::OutOfMemory`] when the node table fills up.
    pub(crate) fn symba(&self, node_limit: usize) -> Result<(Option<Vec<OpId>>, usize), Error> {
        let b = &self.backend;
        let drop_order = self.drop_order();
        let mut next_level = [1_usize; 2];
        let mut installed = 0;

        let forward = Frontier::new(
            b,
            &self.concrete,
            Direction::Forward,
            Partition::zero(b),
            Strategy::Cheapest,
        )?;
        let backward = Frontier::new(
            b,
            &self.concrete,
            Direction::Backward,
            Partition::zero(b),
            Strategy::Cheapest,
        )?;

        let mut hook = |frontier: &mut Frontier<'_, B>| -> Result<(), Error> {
            let level = &mut next_level[usize::from(frontier.direction() == Direction::Backward)];
            if frontier.next_size() <= node_limit || *level >= drop_order.len() {
                return Ok(());
            }

            let space = self.project(&drop_order[..*level])?;
            *level += 1;
            let perimeter = self.perimeter(&space, frontier.direction().reverse(), node_limit)?;
            if perimeter.is_blind(b) {
                return Ok(());
            }

            let combined = frontier.heuristic().max(b, &perimeter)?;
            frontier.set_heuristic(combined)?;
            installed += 1;

            Ok(())
        };

        let plan = self.bidirectional(forward, backward, &mut hook)?;

        Ok((plan, installed))
    }

    /// The variables in the order abstractions drop them: non-goal variables
    /// first, each group from the last variable id down.
    fn drop_order(&self) -> Vec<VarId> {
        let is_goal = |var: &VarId| self.goal_vars.binary_search(var).is_ok();
        let all = (0..self.encoding.num_vars()).rev();

        all.clone()
            .filter(|var| !is_goal(var))
            .chain(all.filter(is_goal))
            .collect()
    }
}
