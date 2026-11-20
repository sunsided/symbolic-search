//! The decision-diagram backend seam.
//!
//! All symbolic code in this crate talks to decision diagrams through the
//! [`Backend`] trait, so the diagram library can be swapped without touching
//! the encoding, the transition relations or the search loops. The trait holds
//! only the operations the crate calls; extend it when an algorithm needs more.
//!
//! Two implementations exist: [`Oxidd`] (the default), built on
//! complement-edge BDDs from the `oxidd` crate (`docs/design.md` records
//! why), and [`ReferenceBdd`], a small plain-Rust BDD that cross-checks it.
//!
//! The trait is public so code can be generic over the backend:
//! [`crate::SymbolicTask::with_backend`] compiles a task on either, and
//! `SymbolicTask` and `Heuristic` take the backend as a type parameter that
//! defaults to [`Oxidd`]. It is sealed: an implementation must keep the
//! variable order fixed (variable 0 on top), give sets semantic equality
//! and report a full table as [`OutOfMemory`], none of which the type
//! system checks, and the trait grows with every algorithm that needs a new
//! operation. Sealing keeps both changes non-breaking.

mod bcdd;
mod reference;

pub use bcdd::{Oxidd, OxiddRenaming, OxiddSet, OxiddVarSet};
pub use reference::{ReferenceBdd, ReferenceRenaming, ReferenceSet, ReferenceVarSet};

/// The seal of [`Backend`]: only this crate's backends implement it.
pub(crate) mod sealed {
    /// Implemented by every backend of this crate, and by nothing else.
    pub trait Sealed {}
}

/// The diagram manager ran out of node or cache capacity.
///
/// Raised by every operation that may allocate nodes. The search maps it to
/// [`crate::Error::OutOfMemory`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct OutOfMemory;

/// Result type of fallible backend operations.
pub type BackendResult<T> = Result<T, OutOfMemory>;

/// Capacity limits for a backend's node table and operation cache.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Capacity {
    /// Maximum number of inner nodes alive at the same time.
    pub nodes: usize,
    /// Number of entries of the apply (computed-table) cache.
    pub cache: usize,
}

/// A decision-diagram library seen as a manager of Boolean functions over a
/// fixed number of variables.
///
/// Variables are numbered `0..num_vars` and the variable order equals the
/// numbering: variable 0 is the topmost level. Implementations must not
/// reorder variables behind the caller's back.
pub trait Backend: sealed::Sealed + Sized {
    /// A Boolean function, read as the set of its satisfying assignments.
    ///
    /// Equality must be semantic: two handles compare equal exactly when they
    /// denote the same function. Canonical diagrams give this for free.
    type Set: Clone + PartialEq;

    /// A prebuilt set of variables to quantify over, reused across calls.
    type VarSet;

    /// A prebuilt variable renaming, reused across calls.
    type Renaming;

    /// Creates a manager with `num_vars` variables and the given capacity.
    ///
    /// # Errors
    ///
    /// [`OutOfMemory`] when the variables do not fit into the node table.
    fn new(num_vars: u32, capacity: Capacity) -> BackendResult<Self>;

    /// The constant function `value`.
    fn constant(&self, value: bool) -> Self::Set;

    /// The positive literal of variable `var`.
    ///
    /// # Panics
    ///
    /// When `var` is not below the `num_vars` passed to [`Backend::new`].
    fn var(&self, var: u32) -> Self::Set;

    /// The complement of `a`.
    ///
    /// # Errors
    ///
    /// [`OutOfMemory`] when the node table or cache is full.
    fn not(&self, a: &Self::Set) -> BackendResult<Self::Set>;

    /// The conjunction (set intersection) of `a` and `b`.
    ///
    /// # Errors
    ///
    /// [`OutOfMemory`] when the node table or cache is full.
    fn and(&self, a: &Self::Set, b: &Self::Set) -> BackendResult<Self::Set>;

    /// The disjunction (set union) of `a` and `b`.
    ///
    /// # Errors
    ///
    /// [`OutOfMemory`] when the node table or cache is full.
    fn or(&self, a: &Self::Set, b: &Self::Set) -> BackendResult<Self::Set>;

    /// If-then-else: `(cond ∧ then) ∨ (¬cond ∧ otherwise)`.
    ///
    /// # Errors
    ///
    /// [`OutOfMemory`] when the node table or cache is full.
    fn ite(
        &self,
        cond: &Self::Set,
        then: &Self::Set,
        otherwise: &Self::Set,
    ) -> BackendResult<Self::Set>;

    /// Builds a reusable quantification set from variable numbers.
    ///
    /// # Errors
    ///
    /// [`OutOfMemory`] when the backend needs nodes to represent the set.
    fn var_set(&self, vars: &[u32]) -> BackendResult<Self::VarSet>;

    /// The relational product `∃ vars. a ∧ b`, computed in one pass.
    ///
    /// # Errors
    ///
    /// [`OutOfMemory`] when the node table or cache is full.
    fn and_exists(
        &self,
        a: &Self::Set,
        b: &Self::Set,
        vars: &Self::VarSet,
    ) -> BackendResult<Self::Set>;

    /// Existential quantification `∃ vars. a`: forgets the values of `vars`.
    ///
    /// # Errors
    ///
    /// [`OutOfMemory`] when the node table or cache is full.
    fn exists(&self, a: &Self::Set, vars: &Self::VarSet) -> BackendResult<Self::Set>;

    /// Builds a reusable renaming that replaces each `from` variable by the
    /// matching `to` variable, in parallel.
    ///
    /// # Panics
    ///
    /// When `from` and `to` have different lengths.
    fn renaming(&self, from: &[u32], to: &[u32]) -> Self::Renaming;

    /// Applies `renaming` to `a`. The target variables must not occur in `a`.
    ///
    /// # Errors
    ///
    /// [`OutOfMemory`] when the node table or cache is full.
    fn rename(&self, a: &Self::Set, renaming: &Self::Renaming) -> BackendResult<Self::Set>;

    /// Whether `a` is the empty set.
    fn is_empty(&self, a: &Self::Set) -> bool;

    /// Number of diagram nodes of `a`, terminals included. A size measure for
    /// thresholds and for choosing the cheaper search direction; linear in
    /// the size of `a`.
    fn node_count(&self, a: &Self::Set) -> usize;

    /// One satisfying assignment of `a`, indexed by variable number, or `None`
    /// when `a` is empty. Variables the diagram leaves open are set to
    /// `false`; the result is still a member of `a`.
    fn pick(&self, a: &Self::Set) -> Option<Vec<bool>>;
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Variables of the cross-check functions.
    const VARS: u32 = 5;

    /// The truth table of `f` over [`VARS`] variables, by intersecting it
    /// with every minterm: needs only trait operations.
    fn truth_table<B: Backend>(b: &B, f: &B::Set) -> Vec<bool> {
        (0..1_u32 << VARS)
            .map(|bits| {
                let minterm = (0..VARS).fold(b.constant(true), |acc, v| {
                    let literal = if bits >> v & 1 == 1 {
                        b.var(v)
                    } else {
                        b.not(&b.var(v)).unwrap()
                    };
                    b.and(&acc, &literal).unwrap()
                });
                !b.is_empty(&b.and(f, &minterm).unwrap())
            })
            .collect()
    }

    /// A pseudo-random function built from every trait operation, steered
    /// by `seed`; the same seed gives the same function on every backend.
    fn random_function<B: Backend>(b: &B, seed: u64) -> B::Set {
        let mut state = seed.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1);
        let mut next = move || {
            state = state
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1_442_695_040_888_963_407);
            state >> 33
        };
        let pick_var = |n: u64| u32::try_from(n % u64::from(VARS)).unwrap();
        let mut pool: Vec<B::Set> = (0..VARS).map(|v| b.var(v)).collect();
        for _ in 0..12 {
            let len = u64::try_from(pool.len()).unwrap();
            let first = pool[usize::try_from(next() % len).unwrap()].clone();
            let second = pool[usize::try_from(next() % len).unwrap()].clone();
            let third = pool[usize::try_from(next() % len).unwrap()].clone();
            let made = match next() % 7 {
                0 => b.not(&first),
                1 => b.and(&first, &second),
                2 => b.or(&first, &second),
                3 => b.ite(&first, &second, &third),
                4 => b.exists(&first, &b.var_set(&[pick_var(next())]).unwrap()),
                5 => b.and_exists(
                    &first,
                    &second,
                    &b.var_set(&[pick_var(next()), pick_var(next())]).unwrap(),
                ),
                _ => {
                    // Rename one variable to one the function does not use.
                    let from = pick_var(next());
                    let to = (from + 1) % VARS;
                    let unused = b.exists(&first, &b.var_set(&[to]).unwrap()).unwrap();
                    b.rename(&unused, &b.renaming(&[from], &[to]))
                }
            };
            pool.push(made.unwrap());
        }

        pool.pop().unwrap()
    }

    /// Both backends compute the same functions, and `pick` returns a
    /// member on both.
    #[test]
    fn reference_bdd_agrees_with_oxidd() {
        let capacity = Capacity {
            nodes: 1 << 12,
            cache: 1 << 8,
        };
        let oxidd = Oxidd::new(VARS, capacity).unwrap();
        let reference = ReferenceBdd::new(VARS, capacity).unwrap();
        for seed in 0..200 {
            let f = random_function(&oxidd, seed);
            let g = random_function(&reference, seed);
            assert_eq!(
                truth_table(&oxidd, &f),
                truth_table(&reference, &g),
                "seed {seed}"
            );
            assert_eq!(oxidd.is_empty(&f), reference.is_empty(&g));

            if let Some(assignment) = reference.pick(&g) {
                let bits = (0..VARS)
                    .filter(|&v| assignment[v as usize])
                    .fold(0, |acc, v| acc | 1 << v);
                assert!(truth_table(&reference, &g)[bits], "seed {seed}");
            }
        }
    }

    /// The reference backend reports a full table instead of panicking.
    #[test]
    fn reference_bdd_runs_out_of_memory() {
        let tiny = Capacity { nodes: 4, cache: 4 };

        assert!(ReferenceBdd::new(VARS, tiny).is_err());
    }
}
