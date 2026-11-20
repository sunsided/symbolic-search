//! [`Backend`] on a small reduced ordered BDD written in plain Rust.
//!
//! A reference implementation, kept simple to be obviously right: no
//! complement edges, no garbage collection, an `ite` computed table that is
//! cleared when full, and quantification and renaming by memoised
//! recursion. It exercises the backend seam and cross-checks [`super::Oxidd`]
//! in the tests; it is far slower and never frees nodes, so the node
//! capacity bounds every node ever created.

use std::cell::RefCell;
use std::collections::HashMap;

use super::{Backend, BackendResult, Capacity, OutOfMemory};

/// Index of the constant-false terminal.
const FALSE: u32 = 0;

/// Index of the constant-true terminal.
const TRUE: u32 = 1;

/// The variable stored in terminal nodes: below every real variable.
const TERMINAL: u32 = u32::MAX;

/// One node: `var ? hi : lo`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
struct Node {
    /// The decision variable; [`TERMINAL`] for the two terminals.
    var: u32,
    /// The child for `var = false`.
    lo: u32,
    /// The child for `var = true`.
    hi: u32,
}

/// The mutable node store behind a [`ReferenceBdd`].
struct Store {
    /// All nodes ever created, terminals at 0 and 1.
    nodes: Vec<Node>,
    /// Hash-consing table from node to index, which keeps diagrams reduced
    /// and canonical.
    unique: HashMap<Node, u32>,
    /// Computed table of `ite(f, g, h)`.
    ite_cache: HashMap<(u32, u32, u32), u32>,
    /// Maximum number of nodes, terminals included.
    capacity: usize,
    /// Maximum number of computed-table entries before it is cleared.
    cache_capacity: usize,
}

/// A reduced ordered BDD manager in plain Rust (see the module docs).
pub struct ReferenceBdd {
    /// The node store; operations take `&self`, so it sits in a cell.
    store: RefCell<Store>,
    /// Number of variables.
    num_vars: u32,
}

/// A set of states on the [`ReferenceBdd`] backend: a node index.
/// Diagrams are canonical, so index equality is semantic equality.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ReferenceSet(u32);

/// A quantification set on the [`ReferenceBdd`] backend: sorted variables.
#[derive(Clone, Debug)]
pub struct ReferenceVarSet(Vec<u32>);

/// A renaming on the [`ReferenceBdd`] backend: source to target variable.
#[derive(Clone, Debug)]
pub struct ReferenceRenaming(HashMap<u32, u32>);

impl Store {
    /// The node `var ? hi : lo`, reduced and hash-consed.
    ///
    /// # Errors
    ///
    /// [`OutOfMemory`] when a new node would exceed the capacity.
    fn mk(&mut self, var: u32, lo: u32, hi: u32) -> BackendResult<u32> {
        if lo == hi {
            return Ok(lo);
        }

        let node = Node { var, lo, hi };
        if let Some(&index) = self.unique.get(&node) {
            return Ok(index);
        }

        if self.nodes.len() >= self.capacity {
            return Err(OutOfMemory);
        }

        let index = u32::try_from(self.nodes.len()).map_err(|_| OutOfMemory)?;
        self.nodes.push(node);
        self.unique.insert(node, index);

        Ok(index)
    }

    /// The cofactors of `f` with respect to `var`: `(f[var=0], f[var=1])`.
    fn cofactors(&self, f: u32, var: u32) -> (u32, u32) {
        let node = self.nodes[f as usize];
        if node.var == var {
            (node.lo, node.hi)
        } else {
            (f, f)
        }
    }

    /// `ite(f, g, h)` by Shannon expansion on the top variable.
    ///
    /// # Errors
    ///
    /// [`OutOfMemory`] when the node table is full.
    fn ite(&mut self, f: u32, g: u32, h: u32) -> BackendResult<u32> {
        match (f, g, h) {
            (TRUE, _, _) => return Ok(g),
            (FALSE, _, _) => return Ok(h),
            _ if g == h => return Ok(g),
            (_, TRUE, FALSE) => return Ok(f),
            _ => {}
        }

        if let Some(&r) = self.ite_cache.get(&(f, g, h)) {
            return Ok(r);
        }

        let var = [f, g, h]
            .iter()
            .map(|&x| self.nodes[x as usize].var)
            .min()
            .expect("three operands");
        let (f0, f1) = self.cofactors(f, var);
        let (g0, g1) = self.cofactors(g, var);
        let (h0, h1) = self.cofactors(h, var);
        let lo = self.ite(f0, g0, h0)?;
        let hi = self.ite(f1, g1, h1)?;
        let r = self.mk(var, lo, hi)?;

        if self.ite_cache.len() >= self.cache_capacity {
            self.ite_cache.clear();
        }
        self.ite_cache.insert((f, g, h), r);

        Ok(r)
    }

    /// `∃ vars. f`, memoised in `memo`. `vars` is sorted.
    ///
    /// # Errors
    ///
    /// [`OutOfMemory`] when the node table is full.
    fn exists(&mut self, f: u32, vars: &[u32], memo: &mut HashMap<u32, u32>) -> BackendResult<u32> {
        let node = self.nodes[f as usize];
        if node.var == TERMINAL || vars.last().is_none_or(|&last| node.var > last) {
            return Ok(f);
        }

        if let Some(&r) = memo.get(&f) {
            return Ok(r);
        }

        let lo = self.exists(node.lo, vars, memo)?;
        let hi = self.exists(node.hi, vars, memo)?;
        let r = if vars.binary_search(&node.var).is_ok() {
            self.ite(lo, TRUE, hi)?
        } else {
            self.mk(node.var, lo, hi)?
        };
        memo.insert(f, r);

        Ok(r)
    }

    /// `f` with every variable renamed by `map`, memoised in `memo`.
    ///
    /// # Errors
    ///
    /// [`OutOfMemory`] when the node table is full.
    fn rename(
        &mut self,
        f: u32,
        map: &HashMap<u32, u32>,
        memo: &mut HashMap<u32, u32>,
    ) -> BackendResult<u32> {
        let node = self.nodes[f as usize];
        if node.var == TERMINAL {
            return Ok(f);
        }

        if let Some(&r) = memo.get(&f) {
            return Ok(r);
        }

        let lo = self.rename(node.lo, map, memo)?;
        let hi = self.rename(node.hi, map, memo)?;
        let var = map.get(&node.var).copied().unwrap_or(node.var);
        let literal = self.mk(var, FALSE, TRUE)?;
        // `ite` restores the order when the renaming moves variables.
        let r = self.ite(literal, hi, lo)?;
        memo.insert(f, r);

        Ok(r)
    }
}

impl super::sealed::Sealed for ReferenceBdd {}

impl Backend for ReferenceBdd {
    type Set = ReferenceSet;
    type VarSet = ReferenceVarSet;
    type Renaming = ReferenceRenaming;

    fn new(num_vars: u32, capacity: Capacity) -> BackendResult<Self> {
        let terminal = |index| Node {
            var: TERMINAL,
            lo: index,
            hi: index,
        };
        let mut store = Store {
            nodes: vec![terminal(FALSE), terminal(TRUE)],
            unique: HashMap::new(),
            ite_cache: HashMap::new(),
            capacity: capacity.nodes,
            cache_capacity: capacity.cache.max(1),
        };
        for var in 0..num_vars {
            store.mk(var, FALSE, TRUE)?;
        }

        Ok(Self {
            store: RefCell::new(store),
            num_vars,
        })
    }

    fn constant(&self, value: bool) -> ReferenceSet {
        ReferenceSet(if value { TRUE } else { FALSE })
    }

    fn var(&self, var: u32) -> ReferenceSet {
        assert!(var < self.num_vars, "variable {var} out of range");
        let node = Node {
            var,
            lo: FALSE,
            hi: TRUE,
        };

        ReferenceSet(self.store.borrow().unique[&node])
    }

    fn not(&self, a: &ReferenceSet) -> BackendResult<ReferenceSet> {
        self.store
            .borrow_mut()
            .ite(a.0, FALSE, TRUE)
            .map(ReferenceSet)
    }

    fn and(&self, a: &ReferenceSet, b: &ReferenceSet) -> BackendResult<ReferenceSet> {
        self.store
            .borrow_mut()
            .ite(a.0, b.0, FALSE)
            .map(ReferenceSet)
    }

    fn or(&self, a: &ReferenceSet, b: &ReferenceSet) -> BackendResult<ReferenceSet> {
        self.store
            .borrow_mut()
            .ite(a.0, TRUE, b.0)
            .map(ReferenceSet)
    }

    fn ite(
        &self,
        cond: &ReferenceSet,
        then: &ReferenceSet,
        otherwise: &ReferenceSet,
    ) -> BackendResult<ReferenceSet> {
        self.store
            .borrow_mut()
            .ite(cond.0, then.0, otherwise.0)
            .map(ReferenceSet)
    }

    fn var_set(&self, vars: &[u32]) -> BackendResult<ReferenceVarSet> {
        let mut vars = vars.to_vec();
        vars.sort_unstable();
        vars.dedup();

        Ok(ReferenceVarSet(vars))
    }

    fn and_exists(
        &self,
        a: &ReferenceSet,
        b: &ReferenceSet,
        vars: &ReferenceVarSet,
    ) -> BackendResult<ReferenceSet> {
        // shortcut: not fused; conjoin, then quantify. Fine for a reference.
        let both = self.and(a, b)?;

        self.exists(&both, vars)
    }

    fn exists(&self, a: &ReferenceSet, vars: &ReferenceVarSet) -> BackendResult<ReferenceSet> {
        self.store
            .borrow_mut()
            .exists(a.0, &vars.0, &mut HashMap::new())
            .map(ReferenceSet)
    }

    fn renaming(&self, from: &[u32], to: &[u32]) -> ReferenceRenaming {
        assert_eq!(from.len(), to.len(), "renaming needs one target per source");
        ReferenceRenaming(from.iter().copied().zip(to.iter().copied()).collect())
    }

    fn rename(
        &self,
        a: &ReferenceSet,
        renaming: &ReferenceRenaming,
    ) -> BackendResult<ReferenceSet> {
        self.store
            .borrow_mut()
            .rename(a.0, &renaming.0, &mut HashMap::new())
            .map(ReferenceSet)
    }

    fn is_empty(&self, a: &ReferenceSet) -> bool {
        a.0 == FALSE
    }

    fn node_count(&self, a: &ReferenceSet) -> usize {
        let store = self.store.borrow();
        let mut seen = std::collections::HashSet::from([a.0]);
        let mut stack = vec![a.0];
        while let Some(f) = stack.pop() {
            let node = store.nodes[f as usize];
            if node.var == TERMINAL {
                continue;
            }

            for child in [node.lo, node.hi] {
                if seen.insert(child) {
                    stack.push(child);
                }
            }
        }

        seen.len()
    }

    fn pick(&self, a: &ReferenceSet) -> Option<Vec<bool>> {
        if a.0 == FALSE {
            return None;
        }

        let store = self.store.borrow();
        let mut assignment = vec![false; self.num_vars as usize];
        let mut f = a.0;
        while f != TRUE {
            let node = store.nodes[f as usize];
            // Reduced diagrams have no path to false only, so a non-false
            // child always leads to true.
            if node.lo == FALSE {
                assignment[node.var as usize] = true;
                f = node.hi;
            } else {
                f = node.lo;
            }
        }

        Some(assignment)
    }
}
