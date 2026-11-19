//! Binary (logarithmic) encoding of SAS+ states.
//!
//! A SAS+ variable with domain size `d` takes `⌈log₂ d⌉` bits; value `k` is
//! stored as the binary code of `k`, most significant bit first. Each bit has
//! a *current* and a *next* copy, and the two copies sit next to each other in
//! the variable order:
//!
//! ```text
//! level:  0      1      2      3      ...
//! bit:    v0.b0  v0.b0' v0.b1  v0.b1' v1.b0  v1.b0' ...
//! ```
//!
//! SAS+ variables keep the task's order. Codes `≥ d` are invalid; every set
//! the search builds stays inside the valid codes (see [`Encoding::valid`]).

use sas_plus::{Fact, State, Task, Value, VarId};

use crate::backend::{Backend, BackendResult};

/// Which copy of the state bits a set talks about.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Frame {
    /// The state before a transition.
    Current,
    /// The state after a transition.
    Next,
}

/// The bits of one SAS+ variable.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Bits {
    /// Global index of the variable's most significant bit, counted over all
    /// SAS+ variables (not a diagram variable number).
    first: u32,
    /// Number of bits, `⌈log₂ domain⌉`. Zero for a single-value domain.
    width: u32,
    /// Domain size of the SAS+ variable. At least 1.
    domain: u32,
}

/// The bit layout of all SAS+ variables of a task.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) struct Encoding {
    /// Indexed by SAS+ variable id.
    vars: Vec<Bits>,
    /// Total number of state bits; the diagram has twice as many variables.
    num_bits: u32,
}

/// Bits needed to store values `0..domain`. Zero for `domain <= 1`.
fn bit_width(domain: u32) -> u32 {
    u32::BITS - domain.saturating_sub(1).leading_zeros()
}

impl Encoding {
    /// Lays out the bits of every variable of `task`, in task order.
    ///
    /// Returns `None` when the diagram would need more than `u32::MAX`
    /// variables.
    pub(crate) fn new(task: &Task) -> Option<Self> {
        let mut vars = Vec::with_capacity(task.variables.len());
        let mut num_bits: u32 = 0;
        for var in &task.variables {
            let domain = var.domain_size.max(1);
            let width = bit_width(domain);
            vars.push(Bits {
                first: num_bits,
                width,
                domain,
            });
            num_bits = num_bits.checked_add(width)?;
        }

        num_bits.checked_mul(2)?;

        Some(Self { vars, num_bits })
    }

    /// Number of diagram variables: a current and a next copy per bit.
    pub(crate) fn num_diagram_vars(&self) -> u32 {
        self.num_bits * 2
    }

    /// Diagram variables of `var` in `frame`, most significant bit first.
    pub(crate) fn diagram_vars(&self, var: VarId, frame: Frame) -> impl Iterator<Item = u32> + '_ {
        let bits = self.vars[var];
        let offset = u32::from(frame == Frame::Next);

        (bits.first..bits.first + bits.width).map(move |bit| 2 * bit + offset)
    }

    /// The set of assignments where `var` has `value` in `frame`.
    ///
    /// # Errors
    ///
    /// Out of memory in the backend.
    pub(crate) fn value<B: Backend>(
        &self,
        b: &B,
        fact: Fact,
        frame: Frame,
    ) -> BackendResult<B::Set> {
        let width = self.vars[fact.var].width;
        let mut set = b.constant(true);
        for (i, v) in self.diagram_vars(fact.var, frame).enumerate() {
            let shift = width - 1 - u32::try_from(i).expect("a variable has at most 32 bits");
            let literal = if (fact.value >> shift) & 1 == 1 {
                b.var(v)
            } else {
                b.not(&b.var(v))?
            };
            set = b.and(&set, &literal)?;
        }

        Ok(set)
    }

    /// The conjunction of `facts`, all in the current frame.
    ///
    /// # Errors
    ///
    /// Out of memory in the backend.
    pub(crate) fn facts<B: Backend>(&self, b: &B, facts: &[Fact]) -> BackendResult<B::Set> {
        facts.iter().try_fold(b.constant(true), |acc, &fact| {
            b.and(&acc, &self.value(b, fact, Frame::Current)?)
        })
    }

    /// The singleton set of a complete state, in the current frame.
    ///
    /// # Errors
    ///
    /// Out of memory in the backend.
    pub(crate) fn state<B: Backend>(&self, b: &B, state: &State) -> BackendResult<B::Set> {
        let facts: Vec<Fact> = state
            .0
            .iter()
            .enumerate()
            .map(|(var, &value)| Fact::new(var, value))
            .collect();

        self.facts(b, &facts)
    }

    /// The assignments where `var` holds a valid code (below its domain size)
    /// in the current frame.
    ///
    /// # Errors
    ///
    /// Out of memory in the backend.
    pub(crate) fn valid<B: Backend>(&self, b: &B, var: VarId) -> BackendResult<B::Set> {
        let bits = self.vars[var];
        if u64::from(bits.domain) == 1_u64 << bits.width {
            return Ok(b.constant(true));
        }

        // shortcut: one cube per value, fine for small domains; use a `< d` comparator for large ones.
        (0..bits.domain).try_fold(b.constant(false), |acc, value| {
            b.or(&acc, &self.value(b, Fact::new(var, value), Frame::Current)?)
        })
    }

    /// The assignments where every variable holds a valid code in the
    /// current frame.
    ///
    /// # Errors
    ///
    /// Out of memory in the backend.
    pub(crate) fn all_valid<B: Backend>(&self, b: &B) -> BackendResult<B::Set> {
        (0..self.vars.len()).try_fold(b.constant(true), |acc, var| {
            b.and(&acc, &self.valid(b, var)?)
        })
    }

    /// The frame axiom of `var`: its next copy equals its current copy.
    ///
    /// # Errors
    ///
    /// Out of memory in the backend.
    pub(crate) fn unchanged<B: Backend>(&self, b: &B, var: VarId) -> BackendResult<B::Set> {
        let current = self.diagram_vars(var, Frame::Current);
        let next = self.diagram_vars(var, Frame::Next);

        current.zip(next).try_fold(b.constant(true), |acc, (c, n)| {
            let next = b.var(n);
            let equal = b.ite(&b.var(c), &next, &b.not(&next)?)?;
            b.and(&acc, &equal)
        })
    }

    /// Decodes the current-frame part of a full diagram assignment into a
    /// SAS+ state.
    ///
    /// # Panics
    ///
    /// When `assignment` is shorter than [`Encoding::num_diagram_vars`].
    pub(crate) fn decode(&self, assignment: &[bool]) -> State {
        let values = (0..self.vars.len())
            .map(|var| {
                self.diagram_vars(var, Frame::Current)
                    .fold(0, |acc: Value, v| {
                        (acc << 1) | Value::from(assignment[v as usize])
                    })
            })
            .collect();

        State(values)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Widths for the boundary domain sizes.
    #[test]
    fn bit_widths() {
        let widths: Vec<u32> = [1, 2, 3, 4, 5, 8, 9].map(bit_width).to_vec();

        assert_eq!(widths, [0, 1, 2, 2, 3, 3, 4]);
    }
}
