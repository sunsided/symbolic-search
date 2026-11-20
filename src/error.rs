//! Errors of building and searching a symbolic task.

use core::fmt;

use sas_plus::{Fact, TaskError, VarId};

use crate::backend::OutOfMemory;

/// Why a symbolic task could not be built or searched.
#[derive(Clone, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum Error {
    /// [`sas_plus::Task::validate`] rejected the task.
    InvalidTask(TaskError),
    /// The task has axioms or derived variables, which the symbolic encoding
    /// does not support yet.
    Axioms,
    /// The binary encoding needs more than `u32::MAX` decision-diagram
    /// variables.
    TooManyBits,
    /// The decision-diagram node table or cache is full, even after garbage
    /// collection. Raise [`crate::Config::node_capacity`].
    OutOfMemory,
    /// A pattern names a variable the task does not have.
    UnknownVariable(VarId),
    /// A fact names a variable the task does not have, or a value outside
    /// that variable's domain.
    InvalidFact(Fact),
    /// A [`crate::Heuristic`] was used with a task other than the one that
    /// built it.
    ForeignHeuristic,
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidTask(e) => write!(f, "invalid task: {e}"),
            Self::Axioms => write!(f, "axioms and derived variables are not supported"),
            Self::TooManyBits => write!(f, "the task needs too many decision-diagram variables"),
            Self::OutOfMemory => write!(f, "the decision-diagram node table is full"),
            Self::UnknownVariable(var) => write!(f, "unknown variable {var} in a pattern"),
            Self::InvalidFact(fact) => {
                write!(f, "invalid fact: variable {} = {}", fact.var, fact.value)
            }
            Self::ForeignHeuristic => write!(f, "the heuristic belongs to another symbolic task"),
        }
    }
}

impl core::error::Error for Error {
    fn source(&self) -> Option<&(dyn core::error::Error + 'static)> {
        match self {
            Self::InvalidTask(e) => Some(e),
            _ => None,
        }
    }
}

impl From<OutOfMemory> for Error {
    fn from(_: OutOfMemory) -> Self {
        Self::OutOfMemory
    }
}
