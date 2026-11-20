//! Static orders of the SAS+ variables in the decision diagram.
//!
//! BDD size depends heavily on the variable order. Variables that constrain
//! each other should sit close together. Gamer (Kissmann and Edelkamp 2011)
//! places the variables of every causal-graph arc close together by
//! minimising
//!
//! ```text
//! Σ_{(u, v) ∈ causal graph} (pos(u) − pos(v))²
//! ```
//!
//! Gamer optimises this from random starts. This module starts from the
//! task order and runs deterministic swap hill climbing, so results do not
//! depend on a random seed.

use sas_plus::{CausalGraph, Task, VarId};

/// How the SAS+ variables are ordered in the decision diagram.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[non_exhaustive]
pub enum VariableOrder {
    /// Keep the task's variable order.
    Task,
    /// Gamer's causal-graph order: minimise the sum of squared distances
    /// between variables joined by a causal-graph arc.
    #[default]
    Gamer,
}

/// Maximum number of improving sweeps over all variable pairs.
///
/// Each sweep costs `O(n² · degree)`; the climb usually converges within a
/// handful of sweeps.
const MAX_SWEEPS: usize = 32;

/// The variables of `task` in diagram order, topmost first.
pub(crate) fn order(task: &Task, kind: VariableOrder) -> Vec<VarId> {
    let task_order: Vec<VarId> = (0..task.num_variables()).collect();
    match kind {
        VariableOrder::Task => task_order,
        VariableOrder::Gamer => gamer(task, task_order),
    }
}

/// Swap hill climbing on the Gamer objective, starting from `start`.
fn gamer(task: &Task, start: Vec<VarId>) -> Vec<VarId> {
    let neighbours = undirected_causal_graph(task);
    let mut order = start;
    let mut position: Vec<usize> = vec![0; order.len()];
    for (pos, &var) in order.iter().enumerate() {
        position[var] = pos;
    }

    for _ in 0..MAX_SWEEPS {
        if !improving_sweep(&neighbours, &mut order, &mut position) {
            break;
        }
    }

    order
}

/// The causal graph as symmetric neighbour lists, without self-loops.
fn undirected_causal_graph(task: &Task) -> Vec<Vec<VarId>> {
    let graph = CausalGraph::new(task);

    (0..task.num_variables())
        .map(|var| {
            let mut all: Vec<VarId> = graph
                .successors(var)
                .iter()
                .chain(graph.predecessors(var))
                .copied()
                .collect();
            all.sort_unstable();
            all.dedup();
            all
        })
        .collect()
}

/// Tries every swap of two positions once and keeps each swap that lowers
/// the objective. Returns whether anything changed.
fn improving_sweep(neighbours: &[Vec<VarId>], order: &mut [VarId], position: &mut [usize]) -> bool {
    let mut improved = false;
    for i in 0..order.len() {
        for j in i + 1..order.len() {
            let (u, v) = (order[i], order[j]);
            if swap_gain(neighbours, position, u, v) > 0 {
                order.swap(i, j);
                position.swap(u, v);
                improved = true;
            }
        }
    }

    improved
}

/// How much the objective drops when `u` and `v` trade positions.
fn swap_gain(neighbours: &[Vec<VarId>], position: &[usize], u: VarId, v: VarId) -> i64 {
    let cost = |var: VarId, at: usize, other: VarId, other_at: usize| -> i64 {
        neighbours[var]
            .iter()
            .map(|&w| {
                // The arc between `u` and `v` keeps its length under a swap.
                let w_at = if w == other { other_at } else { position[w] };
                let d = i64::try_from(at.abs_diff(w_at)).expect("positions fit in i64");
                d * d
            })
            .sum()
    };
    let (pu, pv) = (position[u], position[v]);
    let before = cost(u, pu, v, pv) + cost(v, pv, u, pu);
    let after = cost(u, pv, v, pu) + cost(v, pu, u, pv);

    before - after
}

#[cfg(test)]
mod tests {
    use sas_plus::{Effect, Fact, Operator, State, Variable};

    use super::*;

    /// The Gamer objective of `order` on `neighbours`, each arc counted twice.
    fn objective(neighbours: &[Vec<VarId>], order: &[VarId]) -> usize {
        let mut position = vec![0; order.len()];
        for (pos, &var) in order.iter().enumerate() {
            position[var] = pos;
        }

        neighbours
            .iter()
            .enumerate()
            .flat_map(|(u, ws)| ws.iter().map(move |&w| (u, w)))
            .map(|(u, w)| position[u].abs_diff(position[w]).pow(2))
            .sum()
    }

    /// A task whose only arc joins the first and the last of four variables.
    fn far_apart() -> Task {
        Task {
            variables: (0..4).map(|i| Variable::new(format!("v{i}"), 2)).collect(),
            operators: vec![Operator {
                name: "link".into(),
                pre: vec![Fact::new(0, 0)],
                eff: vec![Effect::new(Fact::new(3, 1))],
                cost: 1,
            }],
            initial: State(vec![0; 4]),
            goal: vec![Fact::new(3, 1)],
            axioms: vec![],
        }
    }

    /// Gamer pulls the two linked variables next to each other.
    #[test]
    fn gamer_places_linked_variables_adjacent() {
        let task = far_apart();
        let neighbours = undirected_causal_graph(&task);

        let order = order(&task, VariableOrder::Gamer);

        assert_eq!(objective(&neighbours, &order), 2);
        let mut sorted = order.clone();
        sorted.sort_unstable();
        assert_eq!(sorted, [0, 1, 2, 3]);
    }

    /// The task order is the identity.
    #[test]
    fn task_order_is_identity() {
        assert_eq!(order(&far_apart(), VariableOrder::Task), [0, 1, 2, 3]);
    }
}
