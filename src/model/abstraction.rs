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
        let whole: Vec<(VarId, usize)> = dropped.iter().map(|&var| (var, usize::MAX)).collect();

        self.project_bits(&whole)
    }

    /// The concrete space with the `n` lowest bits of each `(var, n)` in
    /// `dropped` forgotten (all bits when `n` is at least the width).
    ///
    /// Dropping the low bits of a variable merges its values in blocks of
    /// `2ⁿ` (`0..2ⁿ`, `2ⁿ..2·2ⁿ`, …): a domain abstraction that shrinks the
    /// variable instead of removing it. Like dropping whole variables, it
    /// is the exact induced abstraction, so its distances are admissible and
    /// consistent.
    ///
    /// # Errors
    ///
    /// Out of memory in the backend.
    pub(crate) fn project_bits(&self, dropped: &[(VarId, usize)]) -> BackendResult<Space<B>> {
        let (b, encoding) = (&self.backend, &self.encoding);
        let low = |var: VarId, n: usize, frame| {
            let bits: Vec<u32> = encoding.diagram_vars(var, frame).collect();
            let keep = bits.len().saturating_sub(n);
            bits[keep..].to_vec()
        };
        let bits: Vec<u32> = dropped
            .iter()
            .flat_map(|&(var, n)| {
                let mut bits = low(var, n, Frame::Current);
                bits.extend(low(var, n, Frame::Next));
                bits
            })
            .collect();
        let quantified = b.var_set(&bits)?;
        let valid = dropped
            .iter()
            .filter(|&&(_, n)| n > 0)
            .try_fold(b.constant(true), |acc, &(var, _)| {
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

    /// The canonical heuristic of the PDBs of `patterns`: the maximum, over
    /// all maximal sets of pairwise additive patterns, of the sum of their
    /// PDBs. Two patterns are additive when no operator affects a variable
    /// of each, so every operator's cost counts in at most one of them. The
    /// result is admissible and consistent, and at least the maximum of the
    /// single PDBs. Patterns must hold valid variable ids; no patterns give
    /// the blind heuristic.
    ///
    /// When the additivity graph has more than `max_subsets` maximal
    /// cliques (their number can grow exponentially), the enumeration stops
    /// and the result falls back to the maximum of the single PDBs.
    ///
    /// # Errors
    ///
    /// Out of memory in the backend.
    pub(crate) fn canonical(
        &self,
        patterns: &[Vec<VarId>],
        max_subsets: usize,
    ) -> BackendResult<Partition<B>> {
        let b = &self.backend;
        let pdbs = patterns
            .iter()
            .map(|pattern| self.pdb(pattern))
            .collect::<BackendResult<Vec<_>>>()?;

        let singles = || (0..patterns.len()).map(|i| vec![i]).collect();
        let cliques =
            maximal_cliques(&self.additivity(patterns), max_subsets).unwrap_or_else(singles);

        let mut best: Option<Partition<B>> = None;
        for clique in cliques {
            let mut sum = pdbs[clique[0]].clone();
            for &i in &clique[1..] {
                sum = sum.sum(b, &pdbs[i])?;
            }

            best = Some(match best {
                Some(best) => best.max(b, &sum)?,
                None => sum,
            });
        }

        Ok(best.unwrap_or_else(|| Partition::zero(b)))
    }

    /// `additive[i][j]`: no operator affects a variable of both patterns `i`
    /// and `j` (`i ≠ j`).
    fn additivity(&self, patterns: &[Vec<VarId>]) -> Vec<Vec<bool>> {
        let affecting = |pattern: &[VarId]| -> Vec<bool> {
            self.operators
                .iter()
                .map(|op| op.affected().iter().any(|var| pattern.contains(var)))
                .collect()
        };
        let touched: Vec<Vec<bool>> = patterns.iter().map(|p| affecting(p)).collect();

        (0..patterns.len())
            .map(|i| {
                (0..patterns.len())
                    .map(|j| i != j && !touched[i].iter().zip(&touched[j]).any(|(a, b)| *a && *b))
                    .collect()
            })
            .collect()
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

/// The maximal cliques of the undirected graph `adjacent` (a symmetric
/// matrix), by Bron–Kerbosch with pivoting, or `None` when there are more
/// than `limit`. Every vertex is in at least one clique; isolated vertices
/// are cliques of one.
fn maximal_cliques(adjacent: &[Vec<bool>], limit: usize) -> Option<Vec<Vec<usize>>> {
    let mut cliques = Vec::new();
    let complete = bron_kerbosch(
        adjacent,
        &mut Vec::new(),
        (0..adjacent.len()).collect(),
        Vec::new(),
        &mut Cliques {
            found: &mut cliques,
            limit,
        },
    );

    complete.then_some(cliques)
}

/// Where [`bron_kerbosch`] records cliques, and how many it may record.
struct Cliques<'c> {
    /// The maximal cliques found so far.
    found: &'c mut Vec<Vec<usize>>,
    /// The most cliques to record before giving up.
    limit: usize,
}

/// One Bron–Kerbosch step: extends `clique` by candidates `p`, skipping
/// cliques already covered by `x`, and records maximal ones in `out`.
/// Returns `false` once `out` would exceed its limit.
fn bron_kerbosch(
    adjacent: &[Vec<bool>],
    clique: &mut Vec<usize>,
    p: Vec<usize>,
    mut x: Vec<usize>,
    out: &mut Cliques<'_>,
) -> bool {
    if p.is_empty() && x.is_empty() {
        if out.found.len() >= out.limit {
            return false;
        }

        out.found.push(clique.clone());
        return true;
    }

    let pivot = p
        .iter()
        .chain(&x)
        .copied()
        .max_by_key(|&u| p.iter().filter(|&&v| adjacent[u][v]).count());
    let mut p = p;
    let candidates: Vec<usize> = p
        .iter()
        .copied()
        .filter(|&v| pivot.is_none_or(|u| !adjacent[u][v]))
        .collect();
    for v in candidates {
        let neighbours = |set: &[usize]| set.iter().copied().filter(|&w| adjacent[v][w]).collect();
        clique.push(v);
        let complete = bron_kerbosch(adjacent, clique, neighbours(&p), neighbours(&x), out);
        clique.pop();
        if !complete {
            return false;
        }

        p.retain(|&w| w != v);
        x.push(v);
    }

    true
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::backend::Oxidd;
    use crate::{AbstractionHierarchy, Config};

    /// Tiny tables and a random task with conditional effects and wide
    /// domains, so variables have several bits.
    fn model(seed: u64) -> Model<Oxidd> {
        let params = sas_plus::random::RandomTaskParams {
            max_domain: 7,
            max_conditions: 2,
            ..sas_plus::random::RandomTaskParams::default()
        };
        let task = sas_plus::random::random_task(seed, params);
        let config = Config {
            node_capacity: 1 << 14,
            cache_capacity: 1 << 10,
            ..Config::default()
        };

        Model::new(&task, &config).unwrap()
    }

    /// Whether `lower ≤ upper` on every valid state, with dead ends as
    /// infinity: every state alive under `upper` is alive under `lower`
    /// with a value no larger.
    fn dominated(model: &Model<Oxidd>, lower: &Partition<Oxidd>, upper: &Partition<Oxidd>) -> bool {
        let b = model.backend();
        let valid = model.encoding.all_valid(b).unwrap();
        let alive = |p: &Partition<Oxidd>| {
            p.buckets()
                .iter()
                .fold(b.constant(false), |acc, (_, s)| b.or(&acc, s).unwrap())
        };
        let revived = b
            .and(&alive(upper), &b.not(&alive(lower)).unwrap())
            .unwrap();
        if !b.is_empty(&b.and(&revived, &valid).unwrap()) {
            return false;
        }

        lower.buckets().iter().all(|(hl, sl)| {
            upper.buckets().iter().all(|(hu, su)| {
                let common = b.and(&b.and(sl, su).unwrap(), &valid).unwrap();
                hl <= hu || b.is_empty(&common)
            })
        })
    }

    /// The exact goal distance `h*` of every state, from a backward
    /// Dijkstra to exhaustion in the concrete space.
    fn h_star(model: &Model<Oxidd>) -> Partition<Oxidd> {
        model
            .perimeter(&model.concrete, Direction::Backward, usize::MAX)
            .unwrap()
    }

    /// Every level of the bit-dropping hierarchy gives an admissible PDB,
    /// and each level is at most as informed as the one before.
    #[test]
    fn bit_levels_are_admissible_and_monotone() {
        let mut partial = 0;
        for seed in 0..12 {
            let model = model(seed);
            let exact = h_star(&model);
            let steps = model.drop_steps(AbstractionHierarchy::DropBits);
            let mut finer = exact.clone();
            for level in 1..steps.len() {
                let dropped = crate::model::symba::dropped_bits(&steps[..level]);
                let width = |var| model.encoding.diagram_vars(var, Frame::Current).count();
                partial += usize::from(dropped.iter().any(|&(var, n)| n < width(var)));
                let space = model.project_bits(&dropped).unwrap();
                let pdb = model
                    .perimeter(&space, Direction::Backward, usize::MAX)
                    .unwrap();

                assert!(
                    dominated(&model, &pdb, &exact),
                    "seed {seed}, level {level}"
                );
                assert!(
                    dominated(&model, &pdb, &finer),
                    "seed {seed}, level {level}"
                );
                finer = pdb;
            }
        }

        assert!(partial > 0, "no level kept part of a variable");
    }

    /// Dropping all bits of a variable is the same as dropping the
    /// variable.
    #[test]
    fn dropping_all_bits_drops_the_variable() {
        let model = model(1);
        let b = model.backend();
        let by_var = model.project(&[0]).unwrap();
        let by_bits = model.project_bits(&[(0, usize::MAX)]).unwrap();

        assert!(by_var.goal == by_bits.goal && by_var.initial == by_bits.initial);
        for (x, y) in by_var.groups.iter().zip(&by_bits.groups) {
            let all = model.encoding.all_valid(b).unwrap();
            assert!(x.image(b, &all).unwrap() == y.image(b, &all).unwrap());
        }
    }

    /// The canonical heuristic of singleton patterns is admissible and at
    /// least every single PDB, and strictly better than their maximum on
    /// some task.
    #[test]
    fn canonical_pdbs_are_admissible_and_dominate_single_pdbs() {
        let mut gained = 0;
        for seed in 0..48 {
            let model = model(seed);
            let b = model.backend();
            let patterns: Vec<Vec<VarId>> = (0..model.num_vars()).map(|v| vec![v]).collect();
            let canonical = model.canonical(&patterns, usize::MAX).unwrap();
            let capped = model.canonical(&patterns, 0).unwrap();
            let exact = h_star(&model);
            assert!(dominated(&model, &canonical, &exact), "seed {seed}");

            let mut max = Partition::zero(b);
            for pattern in &patterns {
                let single = model.pdb(pattern).unwrap();
                assert!(dominated(&model, &single, &canonical), "seed {seed}");
                max = max.max(b, &single).unwrap();
            }

            assert!(
                capped.buckets() == max.buckets(),
                "the cap falls back to the maximum"
            );

            gained += usize::from(!dominated(&model, &canonical, &max));
        }

        assert!(gained > 0, "additivity never helped");
    }

    /// Merge-and-shrink without shrinking is `h*`, in both directions'
    /// sense (goal and initial-state distances, against exhaustive
    /// Dijkstras); with shrinking it stays admissible and still informs.
    #[test]
    fn merge_and_shrink_is_exact_unshrunk_and_admissible_shrunk() {
        let (mut informed, mut weaker) = (0, 0);
        for seed in 0..12 {
            let model = model(seed);
            for direction in [Direction::Backward, Direction::Forward] {
                let exact = model
                    .perimeter(&model.concrete, direction, usize::MAX)
                    .unwrap();
                let unshrunk = model
                    .merge_and_shrink_towards(direction, usize::MAX)
                    .unwrap();
                assert!(dominated(&model, &unshrunk, &exact), "seed {seed}");
                assert!(dominated(&model, &exact, &unshrunk), "seed {seed}");

                for limit in [0, 8, 32, 128] {
                    let shrunk = model.merge_and_shrink_towards(direction, limit).unwrap();
                    assert!(
                        dominated(&model, &shrunk, &exact),
                        "seed {seed}, limit {limit}"
                    );
                    informed += usize::from(limit > 0 && !shrunk.is_blind(model.backend()));
                    weaker += usize::from(!dominated(&model, &exact, &shrunk));
                }
            }
        }

        assert!(informed > 0, "shrunk abstractions are all blind");
        assert!(weaker > 0, "shrinking never lost information");
    }

    /// Hand-checked cliques: a triangle `0-1-2`, an edge `2-3`, and the
    /// isolated vertex 4.
    #[test]
    fn maximal_cliques_of_a_small_graph() {
        let edges = [(0, 1), (0, 2), (1, 2), (2, 3)];
        let mut adjacent = vec![vec![false; 5]; 5];
        for (u, v) in edges {
            adjacent[u][v] = true;
            adjacent[v][u] = true;
        }

        assert_eq!(maximal_cliques(&adjacent, 2), None);
        let mut cliques: Vec<Vec<usize>> = maximal_cliques(&adjacent, 3)
            .unwrap()
            .into_iter()
            .map(|mut c| {
                c.sort_unstable();
                c
            })
            .collect();
        cliques.sort();

        assert_eq!(cliques, [vec![0, 1, 2], vec![2, 3], vec![4]]);
    }
}
