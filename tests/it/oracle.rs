//! Explicit-state breadth-first search and Dijkstra, the references for plan
//! lengths and costs.

use std::cmp::Reverse;
use std::collections::{BinaryHeap, HashMap, HashSet, VecDeque};

use sas_plus::{State, Task};

/// Number of operators in a shortest plan of `task`, ignoring costs, or
/// `None` when no goal state is reachable. Explores the full reachable state
/// space in the worst case, so only for tiny tasks.
pub fn shortest_plan_length(task: &Task) -> Option<usize> {
    let initial = task.initial_state();
    let mut seen = HashSet::from([initial.clone()]);
    let mut queue = VecDeque::from([(initial, 0)]);
    while let Some((state, depth)) = queue.pop_front() {
        if task.is_goal(&state) {
            return Some(depth);
        }

        for op in task.applicable_operators(&state) {
            let next = task.apply(op, &state);
            if seen.insert(next.clone()) {
                queue.push_back((next, depth + 1));
            }
        }
    }

    None
}

/// Cost of a cheapest plan of `task` by explicit-state Dijkstra, or `None`
/// when no goal state is reachable. Only for tiny tasks.
pub fn cheapest_plan_cost(task: &Task) -> Option<u64> {
    let initial = task.initial_state();
    let mut best = HashMap::from([(initial.clone(), 0)]);
    let mut queue = BinaryHeap::from([(Reverse(0), initial.0)]);
    while let Some((Reverse(g), values)) = queue.pop() {
        let state = State(values);
        if best[&state] < g {
            continue;
        }

        if task.is_goal(&state) {
            return Some(g);
        }

        for op in task.applicable_operators(&state) {
            let next = task.apply(op, &state);
            let cost = g + u64::from(task.operators[op].cost);
            if best.get(&next).is_none_or(|&old| cost < old) {
                best.insert(next.clone(), cost);
                queue.push((Reverse(cost), next.0));
            }
        }
    }

    None
}

/// Every state reachable from the initial state of `task`, in BFS order.
pub fn reachable_states(task: &Task) -> Vec<State> {
    let initial = task.initial_state();
    let mut seen = HashSet::from([initial.clone()]);
    let mut order = vec![initial];
    let mut next = 0;
    while let Some(state) = order.get(next).cloned() {
        next += 1;
        for op in task.applicable_operators(&state) {
            let successor = task.apply(op, &state);
            if seen.insert(successor.clone()) {
                order.push(successor);
            }
        }
    }

    order
}
