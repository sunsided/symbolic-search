//! The decision-diagram backend seam.
//!
//! All symbolic code in this crate talks to decision diagrams through the
//! [`Backend`] trait, so the diagram library can be swapped without touching
//! the encoding, the transition relations or the search loops. The trait holds
//! only the operations the crate calls; extend it when an algorithm needs more.
//!
//! The default (and so far only) implementation is [`bcdd::Oxidd`], built on
//! complement-edge BDDs from the `oxidd` crate. `docs/design.md` records why.

pub(crate) mod bcdd;

/// The diagram manager ran out of node or cache capacity.
///
/// Raised by every operation that may allocate nodes. The search maps it to
/// [`crate::Error::OutOfMemory`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct OutOfMemory;

/// Result type of fallible backend operations.
pub(crate) type BackendResult<T> = Result<T, OutOfMemory>;

/// Capacity limits for a backend's node table and operation cache.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct Capacity {
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
pub(crate) trait Backend: Sized {
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
    fn not(&self, a: &Self::Set) -> BackendResult<Self::Set>;

    /// The conjunction (set intersection) of `a` and `b`.
    fn and(&self, a: &Self::Set, b: &Self::Set) -> BackendResult<Self::Set>;

    /// The disjunction (set union) of `a` and `b`.
    fn or(&self, a: &Self::Set, b: &Self::Set) -> BackendResult<Self::Set>;

    /// If-then-else: `(cond ∧ then) ∨ (¬cond ∧ otherwise)`.
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
    fn and_exists(
        &self,
        a: &Self::Set,
        b: &Self::Set,
        vars: &Self::VarSet,
    ) -> BackendResult<Self::Set>;

    /// Builds a reusable renaming that replaces each `from` variable by the
    /// matching `to` variable, in parallel.
    ///
    /// # Panics
    ///
    /// When `from` and `to` have different lengths.
    fn renaming(&self, from: &[u32], to: &[u32]) -> Self::Renaming;

    /// Applies `renaming` to `a`. The target variables must not occur in `a`.
    fn rename(&self, a: &Self::Set, renaming: &Self::Renaming) -> BackendResult<Self::Set>;

    /// Whether `a` is the empty set.
    fn is_empty(&self, a: &Self::Set) -> bool;

    /// One satisfying assignment of `a`, indexed by variable number, or `None`
    /// when `a` is empty. Variables the diagram leaves open are set to
    /// `false`; the result is still a member of `a`.
    fn pick(&self, a: &Self::Set) -> Option<Vec<bool>>;
}
