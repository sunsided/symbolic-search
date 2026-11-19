//! Explicit-state breadth-first search, the reference for plan lengths.

use std::collections::{HashSet, VecDeque};

use sas_plus::Task;

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
