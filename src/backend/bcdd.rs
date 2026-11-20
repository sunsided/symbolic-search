//! [`Backend`] on complement-edge BDDs from the `oxidd` crate.
//!
//! Uses the index-based manager with a fixed node capacity, a direct-mapped
//! apply cache and no worker threads. The capacity is a hard limit: an
//! operation that needs more nodes than fit fails with [`OutOfMemory`] after
//! garbage collection, instead of growing the table.

use oxidd::bcdd::{BCDDFunction, BCDDManagerRef};
use oxidd::util::{AllocResult, OptBool};
use oxidd::{
    BooleanFunction, BooleanFunctionQuant, BooleanOperator, Function, FunctionSubst, Manager,
    ManagerRef, Subst,
};

use super::{Backend, BackendResult, Capacity, OutOfMemory};

/// A BCDD manager plus the literal of every variable, created up front.
pub struct Oxidd {
    /// The shared manager that owns all nodes of this search.
    manager: BCDDManagerRef,
    /// `literals[i]` is the positive literal of variable `i`.
    literals: Vec<BCDDFunction>,
}

/// Maps an `oxidd` allocation failure onto the backend error.
fn checked<T>(result: AllocResult<T>) -> BackendResult<T> {
    result.map_err(|_| OutOfMemory)
}

impl Backend for Oxidd {
    type Set = BCDDFunction;
    type VarSet = BCDDFunction;
    type Renaming = Subst<BCDDFunction>;

    fn new(num_vars: u32, capacity: Capacity) -> BackendResult<Self> {
        // One thread: the `multi-threading` feature is off.
        let manager = oxidd::bcdd::new_manager(capacity.nodes, capacity.cache, 1);
        let literals = manager.with_manager_exclusive(|m| {
            AllocResult::from_iter(m.add_vars(num_vars).map(|v| BCDDFunction::var(m, v)))
        });

        Ok(Self {
            manager,
            literals: checked(literals)?,
        })
    }

    fn constant(&self, value: bool) -> Self::Set {
        self.manager.with_manager_shared(|m| {
            if value {
                BCDDFunction::t(m)
            } else {
                BCDDFunction::f(m)
            }
        })
    }

    fn var(&self, var: u32) -> Self::Set {
        self.literals[var as usize].clone()
    }

    fn not(&self, a: &Self::Set) -> BackendResult<Self::Set> {
        checked(a.not())
    }

    fn and(&self, a: &Self::Set, b: &Self::Set) -> BackendResult<Self::Set> {
        checked(a.and(b))
    }

    fn or(&self, a: &Self::Set, b: &Self::Set) -> BackendResult<Self::Set> {
        checked(a.or(b))
    }

    fn ite(
        &self,
        cond: &Self::Set,
        then: &Self::Set,
        otherwise: &Self::Set,
    ) -> BackendResult<Self::Set> {
        checked(cond.ite(then, otherwise))
    }

    fn var_set(&self, vars: &[u32]) -> BackendResult<Self::VarSet> {
        // oxidd takes quantified variables as a conjunction of positive literals.
        vars.iter()
            .try_fold(self.constant(true), |acc, &v| self.and(&acc, &self.var(v)))
    }

    fn and_exists(
        &self,
        a: &Self::Set,
        b: &Self::Set,
        vars: &Self::VarSet,
    ) -> BackendResult<Self::Set> {
        checked(a.apply_exists(BooleanOperator::And, b, vars))
    }

    fn exists(&self, a: &Self::Set, vars: &Self::VarSet) -> BackendResult<Self::Set> {
        checked(a.exists(vars))
    }

    fn renaming(&self, from: &[u32], to: &[u32]) -> Self::Renaming {
        assert_eq!(from.len(), to.len(), "renaming needs one target per source");
        Subst::new(from.to_vec(), to.iter().map(|&v| self.var(v)).collect())
    }

    fn rename(&self, a: &Self::Set, renaming: &Self::Renaming) -> BackendResult<Self::Set> {
        checked(a.substitute(renaming))
    }

    fn is_empty(&self, a: &Self::Set) -> bool {
        !a.satisfiable()
    }

    fn node_count(&self, a: &Self::Set) -> usize {
        a.node_count()
    }

    fn pick(&self, a: &Self::Set) -> Option<Vec<bool>> {
        let cube = a.pick_cube(|_, _, _| false)?;
        let mut assignment: Vec<bool> = cube.into_iter().map(|b| b == OptBool::True).collect();
        // The cube may be shorter than the variable count when trailing levels are open.
        assignment.resize(self.literals.len(), false);

        Some(assignment)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Small capacity for unit tests.
    const SMALL: Capacity = Capacity {
        nodes: 256,
        cache: 256,
    };

    /// Image of `x ∧ y` under `x' = ¬x`, quantifying `x` and renaming `x'` to `x`.
    #[test]
    fn relational_product_and_rename() {
        let b = Oxidd::new(4, SMALL).unwrap();
        let (x, xn, y) = (b.var(0), b.var(1), b.var(2));
        let not_x = b.not(&x).unwrap();
        let relation = b.ite(&x, &b.not(&xn).unwrap(), &xn).unwrap();
        let states = b.and(&x, &y).unwrap();

        let product = b.and_exists(&states, &relation, &b.var_set(&[0]).unwrap());
        let image = b.rename(&product.unwrap(), &b.renaming(&[1], &[0]));

        assert!(image.unwrap() == b.and(&not_x, &y).unwrap());
    }

    /// Picking from a cube returns that cube, with open variables false.
    #[test]
    fn pick_fills_open_variables() {
        let b = Oxidd::new(4, SMALL).unwrap();
        let set = b.and(&b.var(2), &b.not(&b.var(3)).unwrap()).unwrap();

        assert_eq!(b.pick(&set), Some(vec![false, false, true, false]));
        assert_eq!(b.pick(&b.constant(false)), None);
    }

    /// Quantifying `x` out of `x ∧ y` leaves `y`, a smaller diagram.
    #[test]
    fn exists_forgets_variables() {
        let b = Oxidd::new(4, SMALL).unwrap();
        let (x, y) = (b.var(0), b.var(2));
        let both = b.and(&x, &y).unwrap();

        let forgotten = b.exists(&both, &b.var_set(&[0]).unwrap()).unwrap();

        assert!(forgotten == y);
        assert!(b.node_count(&both) > b.node_count(&forgotten));
    }

    /// A node table that is too small reports out-of-memory.
    #[test]
    fn exhausted_capacity_is_an_error() {
        let tiny = Capacity { nodes: 8, cache: 8 };

        assert!(Oxidd::new(64, tiny).is_err());
    }
}
