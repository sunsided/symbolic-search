//! How a [`crate::Heuristic`] was built, so it can be built again in a
//! larger manager after an out-of-memory retry.

use sas_plus::{Fact, VarId};

use crate::Error;
use crate::backend::Backend;
use crate::model::Model;
use crate::model::heuristic::Partition;

/// The inputs of one heuristic constructor of [`crate::SymbolicTask`].
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) enum Recipe {
    /// [`crate::SymbolicTask::pdb`] of this pattern.
    Pdb(Vec<VarId>),
    /// [`crate::SymbolicTask::canonical_pdbs_with`] of these patterns.
    Canonical {
        /// The pattern collection.
        patterns: Vec<Vec<VarId>>,
        /// The cap on enumerated additive subsets.
        max_subsets: usize,
    },
    /// [`crate::SymbolicTask::heuristic`] from these estimates.
    Estimates(Vec<(Vec<Fact>, u64)>),
    /// [`crate::SymbolicTask::merge_and_shrink`] with this shrink limit.
    MergeAndShrink(usize),
}

impl Recipe {
    /// Builds the heuristic in `model`'s manager.
    ///
    /// # Errors
    ///
    /// [`Error::InvalidFact`] for a bad estimate, [`Error::OutOfMemory`]
    /// when the node table fills up.
    pub(crate) fn build<B: Backend>(&self, model: &Model<B>) -> Result<Partition<B>, Error> {
        Ok(match self {
            Self::Pdb(pattern) => model.pdb(pattern)?,
            Self::Canonical {
                patterns,
                max_subsets,
            } => model.canonical(patterns, *max_subsets)?,
            Self::Estimates(estimates) => {
                let pairs = estimates
                    .iter()
                    .map(|(facts, h)| Ok((*h, model.facts(facts)?)))
                    .collect::<Result<Vec<_>, Error>>()?;
                Partition::from_overlapping(model.backend(), pairs, Some(0))?
            }
            Self::MergeAndShrink(limit) => model.merge_and_shrink(*limit)?,
        })
    }
}
