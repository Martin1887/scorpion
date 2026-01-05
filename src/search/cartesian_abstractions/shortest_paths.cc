#include "shortest_paths.h"

#include "abstract_state.h"
#include "abstraction.h"
#include "transition_rewirer.h"
#include "utils.h"

#include "../algorithms/priority_queues.h"
#include "../utils/countdown_timer.h"
#include "../utils/logging.h"

#include <cassert>
#include <execution>
#include <map>

using namespace std;

namespace cartesian_abstractions {
const Cost ShortestPaths::INF_COSTS = numeric_limits<Cost>::max();

ShortestPaths::ShortestPaths(
    TransitionRewirer &rewirer,
    const vector<int> &costs,
    int max_cached_spt,
    const utils::CountdownTimer &timer,
    utils::LogProxy &log,
    bool debug)
    : rewirer(rewirer),
      timer(timer),
      log(log),
      max_cached_shortest_paths(max_cached_spt),
      use_cache(max_cached_spt > 0),
      debug(debug),
      task_has_zero_costs(any_of(costs.begin(), costs.end(), [](int c) {return c == 0;})),
      num_cached_shortest_paths(0) {
    operator_costs.reserve(costs.size());
    for (int cost : costs) {
        operator_costs.push_back(convert_to_64_bit_cost(cost));
    }
    if (log.is_at_least_normal()) {
        log << "Subtask has zero-cost operators: " << boolalpha
            << task_has_zero_costs << endl;
    }
}

Cost ShortestPaths::add_costs(Cost a, Cost b) {
    return (a == INF_COSTS || b == INF_COSTS) ? INF_COSTS : a + b;
}

int ShortestPaths::convert_to_actual_cost_for_epsilon_transformed_costs(Cost cost) {
    if (cost == INF_COSTS) {
        return INF;
    } else if (cost > numeric_limits<int>::max()) {
        return static_cast<int>(cost >> 32);
    } else {
        return cost;
    }
}

int ShortestPaths::convert_to_32_bit_cost(Cost cost) const {
    if (cost == INF_COSTS) {
        return INF;
    } else if (task_has_zero_costs) {
        return static_cast<int>(cost >> 32);
    } else {
        return cost;
    }
}

Cost ShortestPaths::convert_to_64_bit_cost(int cost) const {
    assert(cost >= 0);
    if (cost == INF) {
        return INF_COSTS;
    } else if (task_has_zero_costs) {
        if (cost == 0) {
            return 1;
        } else {
            return static_cast<uint64_t>(cost) << 32;
        }
    } else {
        return cost;
    }
}

void ShortestPaths::resize(int num_states) {
    states.resize(num_states);

    if (use_cache && num_cached_shortest_paths > max_cached_shortest_paths) {
        log << "Maximum number of cached shortest paths exceeded --> clear cache." << endl;

        // For each state, remember single arbitrary parent.
        parent.resize(num_states);
        reverse_parent.resize(num_states);
        for (int state = 0; state < static_cast<int>(parents.size()); ++state) {
            if (!parents[state].empty()) {
                parent[state] = parents[state].front();
            }
            if (!reverse_parents[state].empty()) {
                reverse_parent[state] = reverse_parents[state].front();
            }
        }

        // Free memory.
        deque<Transitions>().swap(children);
        deque<Transitions>().swap(parents);
        deque<Transitions>().swap(reverse_children);
        deque<Transitions>().swap(reverse_parents);
        use_cache = false;
    }

    if (use_cache) {
        children.resize(num_states);
        parents.resize(num_states);
        reverse_children.resize(num_states);
        reverse_parents.resize(num_states);
    } else {
        parent.resize(num_states);
        reverse_parent.resize(num_states);
    }
}

void ShortestPaths::recompute(
    const Abstraction &abstraction,
    const Goals &goals,
    const int initial_state) {
    int num_states = abstraction.get_num_states();
    resize(num_states);

    open_queue.clear();
    recompute_forward(abstraction, goals);
    open_queue.clear();
    recompute_backward(abstraction, initial_state);
    assert(test_distances(abstraction, goals));
}
void ShortestPaths::recompute_forward(
    const Abstraction &abstraction,
    const unordered_set<int> &goals) {
    for (StateInfo &state : states) {
        state.goal_distance = INF_COSTS;
    }
    for (int goal : goals) {
        Cost dist = 0;
        states[goal].goal_distance = dist;
        clear_parents(goal);
        open_queue.push(dist, goal);
    }
    while (!open_queue.empty()) {
        pair<Cost, int> top_pair = open_queue.pop();
        Cost old_g = top_pair.first;
        int state_id = top_pair.second;

        Cost g = states[state_id].goal_distance;
        assert(g < INF_COSTS);
        assert(g <= old_g);
        if (g < old_g)
            continue;
        for (const Transition &t : abstraction.get_incoming_transitions(state_id)) {
            int succ_id = t.target_id;
            int op_id = t.op_id;
            Cost op_cost = operator_costs[op_id];
            Cost succ_g = add_costs(g, op_cost);
            if (succ_g < states[succ_id].goal_distance) {
                states[succ_id].goal_distance = succ_g;
                set_parent(succ_id, Transition(op_id, state_id));
                open_queue.push(succ_g, succ_id);
            } else if (use_cache && succ_g != INF_COSTS && succ_g == states[succ_id].goal_distance) {
                add_parent(succ_id, Transition(op_id, state_id));
            }
        }
    }
}

void ShortestPaths::recompute_backward(
    const Abstraction &abstraction,
    const int initial_state) {
    for (StateInfo &state : states) {
        state.init_distance = INF_COSTS;
    }
    Cost dist = 0;
    states[initial_state].init_distance = dist;
    clear_parents(initial_state, true);
    open_queue.push(dist, initial_state);
    while (!open_queue.empty()) {
        pair<Cost, int> top_pair = open_queue.pop();
        Cost old_g = top_pair.first;
        int state_id = top_pair.second;

        Cost g = states[state_id].init_distance;
        assert(g < INF_COSTS);
        assert(g <= old_g);
        if (g < old_g)
            continue;
        for (const Transition &t : abstraction.get_outgoing_transitions(state_id)) {
            int succ_id = t.target_id;
            int op_id = t.op_id;
            Cost op_cost = operator_costs[op_id];
            Cost succ_g = add_costs(g, op_cost);
            if (succ_g < states[succ_id].init_distance) {
                states[succ_id].init_distance = succ_g;
                set_parent(succ_id, Transition(op_id, state_id), true);
                open_queue.push(succ_g, succ_id);
            } else if (use_cache && succ_g != INF_COSTS && succ_g == states[succ_id].init_distance) {
                add_parent(succ_id, Transition(op_id, state_id), true);
            }
        }
    }
}

unique_ptr<Solution> ShortestPaths::extract_solution(
    int init_id, const Goals &goals) {
    // h* = \infty iff goal is unreachable from this state.
    if (states[init_id].goal_distance == INF_COSTS) {
        return nullptr;
    }

    int current_state = init_id;
    unique_ptr<Solution> solution = make_unique<Solution>();
    assert(!goals.count(current_state));
    if (debug) {
        log << "Extract solution" << endl;
    }
    while (!goals.count(current_state)) {
        if (debug) {
            log << "State: " << current_state << endl;
            if (use_cache) {
                log << "Parents: " << parents[current_state] << endl;
                log << "Children: " << children[current_state] << endl;
            } else {
                log << "Parent: " << parent[current_state] << endl;
                log << "Reverse parent: " << reverse_parent[current_state] << endl;
            }
        }
        assert(!use_cache || !parents[current_state].empty());
        // Pick arbitrary parent if there are multiple parents.
        Transition t = use_cache ? parents[current_state].front() : parent[current_state];
        assert(t.is_defined());
        assert(t.target_id != current_state);
        assert(states[t.target_id].goal_distance <= states[current_state].goal_distance);
        solution->push_back(t);
        current_state = t.target_id;
    }
    return solution;
}

vector<int> ShortestPaths::get_goal_distances() const {
    vector<int> distances;
    distances.reserve(states.size());
    for (const StateInfo &state : states) {
        distances.push_back(convert_to_32_bit_cost(state.goal_distance));
    }
    return distances;
}

void ShortestPaths::set_parent(int state, const Transition &new_parent, bool reverse) {
    if (debug) {
        log << "Set parent " << new_parent << " for " << state << endl;
    }
    if (use_cache) {
        clear_parents(state, reverse);
        add_parent(state, new_parent, reverse);
    } else if (reverse) {
        reverse_parent[state] = new_parent;
    } else {
        parent[state] = new_parent;
    }
}

void ShortestPaths::add_parent(int state, const Transition &new_parent, bool reverse) {
    if (reverse) {
        if (debug) {
            log << "Add reverse parent " << new_parent << " for " << state << endl;
        }
        assert(use_cache);
        assert(new_parent.is_defined());
        assert(find(reverse_parents[state].begin(), reverse_parents[state].end(), new_parent) == reverse_parents[state].end());
        reverse_parents[state].push_back(new_parent);
        ++num_cached_shortest_paths;
        Transitions &target_children = reverse_children[new_parent.target_id];
        assert(find(target_children.begin(), target_children.end(),
                    Transition(new_parent.op_id, state)) == target_children.end());
        target_children.emplace_back(new_parent.op_id, state);
    } else {
        if (debug) {
            log << "Add parent " << new_parent << " for " << state << endl;
        }
        assert(use_cache);
        assert(new_parent.is_defined());
        assert(find(parents[state].begin(), parents[state].end(), new_parent) == parents[state].end());
        parents[state].push_back(new_parent);
        ++num_cached_shortest_paths;
        Transitions &target_children = children[new_parent.target_id];
        assert(find(target_children.begin(), target_children.end(),
                    Transition(new_parent.op_id, state)) == target_children.end());
        target_children.emplace_back(new_parent.op_id, state);
    }
}

void ShortestPaths::remove_parent(int state, const Transition &parent, bool reverse) {
    if (reverse) {
        if (debug) {
            log << "Remove reverse parent " << parent << " from " << state << endl;
        }
        assert(use_cache);
        assert(parent.is_defined());
        auto it = find(execution::unseq, reverse_parents[state].begin(), reverse_parents[state].end(), parent);
        assert(it != reverse_parents[state].end());
        utils::swap_and_pop_from_vector(reverse_parents[state], it - reverse_parents[state].begin());
        --num_cached_shortest_paths;
    } else {
        if (debug) {
            log << "Remove parent " << parent << " from " << state << endl;
        }
        assert(use_cache);
        assert(parent.is_defined());
        auto it = find(execution::unseq, parents[state].begin(), parents[state].end(), parent);
        assert(it != parents[state].end());
        utils::swap_and_pop_from_vector(parents[state], it - parents[state].begin());
        --num_cached_shortest_paths;
    }
}

void ShortestPaths::clear_parents(int state, bool reverse) {
    if (reverse) {
        if (debug) {
            log << "Clear reverse parents for " << state << endl;
        }
        if (use_cache) {
            num_cached_shortest_paths -= reverse_parents[state].size();
            while (!reverse_parents[state].empty()) {
                Transition parent = move(reverse_parents[state].back());
                remove_child(parent.target_id, Transition(parent.op_id, state), reverse);
                reverse_parents[state].pop_back();
            }
        } else {
            set_parent(state, Transition(), reverse);
        }
    } else {
        if (debug) {
            log << "Clear parents for " << state << endl;
        }
        if (use_cache) {
            num_cached_shortest_paths -= parents[state].size();
            while (!parents[state].empty()) {
                Transition parent = move(parents[state].back());
                remove_child(parent.target_id, Transition(parent.op_id, state));
                parents[state].pop_back();
            }
        } else {
            set_parent(state, Transition());
        }
    }
}

void ShortestPaths::remove_child(int state, const Transition &child, bool reverse) {
    Transitions &state_children = reverse ? reverse_children[state] : children[state];
    if (debug) {
        if (reverse) {
            log << "Remove reverse child " << child << " from " << state << endl;
        } else {
            log << "Remove child " << child << " from " << state << endl;
        }
    }
    assert(use_cache);
    auto it = find(execution::unseq, state_children.begin(), state_children.end(), child);
    assert(it != state_children.end());
    utils::swap_and_pop_from_vector(state_children, it - state_children.begin());
}

void ShortestPaths::mark_dirty(int state, bool reverse) {
    if (debug) {
        log << "Mark (" << reverse << ") " << state << " as dirty" << endl;
    }
    assert(!use_cache || (reverse ? reverse_parents[state].empty() : parents[state].empty()));
    assert(!count(dirty_states.begin(), dirty_states.end(), state));
    states[state].dirty = true;
    dirty_states.push_back(state);
}

void ShortestPaths::update_incrementally(
    const Abstraction &abstraction,
    int v, int v1, int v2,
    const optional<Transitions> &old_incoming,
    const optional<Transitions> &old_outgoing,
    int var) {
    int num_states = abstraction.get_num_states();
    resize(num_states);
    dirty_states.clear();
    update_incrementally_in_direction(abstraction, v, v1, v2, old_incoming, old_outgoing, var, false);
    dirty_states.clear();
    update_incrementally_in_direction(abstraction, v, v1, v2, old_incoming, old_outgoing, var, true);
    assert(test_distances(abstraction, abstraction.get_goals()));
}

void ShortestPaths::update_incrementally_in_direction(
    const Abstraction &abstraction,
    int v, int v1, int v2,
    const optional<Transitions> &old_incoming,
    const optional<Transitions> &old_outgoing,
    int var,
    bool backward) {
    const unordered_set<int> &goals = abstraction.get_goals();
    const int initial_state = abstraction.get_initial_state_id();
    string target_dist = "Goal ";
    if (backward) {
        target_dist = "Init ";
    }
    if (debug) {
        log << endl << "Reflect splitting " << v << " into " << v1 << " and " << v2
            << (backward ? " in backward_direction": "") << endl;
        log << endl;
        log << "Goals: " << endl;
        for (auto goal : goals) {
            log << goal << endl;
        }
    }

    // Copy distance from split state. Distances will be updated if necessary.
    if (backward) {
        states[v1].init_distance = states[v2].init_distance = states[v].init_distance;
    } else {
        states[v1].goal_distance = states[v2].goal_distance = states[v].goal_distance;
    }
    if (debug) {
        if (backward) {
            for (size_t state = 0; state < reverse_children.size(); ++state) {
                log << state << " children: " << reverse_children[state];
                if (use_cache) {
                    log << endl << state << " parents: " << reverse_parents[state] << endl;
                } else {
                    log << ", parent: " << reverse_parent[state] << endl;
                }
            }
        } else {
            for (size_t state = 0; state < children.size(); ++state) {
                log << state << " children: " << children[state];
                if (use_cache) {
                    log << endl << state << " parents: " << parents[state] << endl;
                } else {
                    log << ", parent: " << parent[state] << endl;
                }
            }
        }
        log << "Reconnect children of split node." << endl;
    }

    /* Update shortest path tree (SPT) transitions to v. The SPT transitions
       will be updated again if v1 or v2 are dirty. */

    if (use_cache) {
        if (backward) {
            num_cached_shortest_paths -= (reverse_children[v].size() + reverse_parents[v].size());
            if (debug) {
                log << "reverse_parents before: ";
                for (const Transition &p : reverse_parents[v]) {
                    log << p.target_id << ", ";
                }
                log << endl;
                log << "reverse_children before: ";
                for (const Transition &p : reverse_children[v]) {
                    log << p.target_id << ", ";
                }
                log << endl;
            }
            rewirer.rewire_transitions(
                reverse_parents, reverse_children, abstraction.get_states(), v,
                abstraction.get_state(v1), abstraction.get_state(v2), var);
            if (debug) {
                log << "reverse_parents after in " << v1 << ": ";
                for (const Transition &p : reverse_parents[v1]) {
                    log << p.target_id << ", ";
                }
                log << endl;
                log << "reverse_children after in " << v1 << ": ";
                for (const Transition &p : reverse_children[v1]) {
                    log << p.target_id << ", ";
                }
                log << endl;
                log << "reverse_parents after in " << v2 << ": ";
                for (const Transition &p : reverse_parents[v2]) {
                    log << p.target_id << ", ";
                }
                log << endl;
                log << "reverse_children after in " << v2 << ": ";
                for (const Transition &p : reverse_children[v2]) {
                    log << p.target_id << ", ";
                }
                log << endl;
            }
            num_cached_shortest_paths +=
                reverse_children[v1].size() + reverse_children[v2].size() +
                reverse_parents[v1].size() + reverse_parents[v2].size();
        } else {
            num_cached_shortest_paths -= (children[v].size() + parents[v].size());
            rewirer.rewire_transitions(
                children, parents, abstraction.get_states(), v,
                abstraction.get_state(v1), abstraction.get_state(v2), var);
            num_cached_shortest_paths +=
                children[v1].size() + children[v2].size() +
                parents[v1].size() + parents[v2].size();
        }
    } else {
        for (int state : {v1, v2}) {
            for (const Transition &transition : backward ?
                 abstraction.get_outgoing_transitions(state) : abstraction.get_incoming_transitions(state)) {
                int u = transition.target_id;
                int op = transition.op_id;
                const Transition &sp = backward ? reverse_parent[u] : parent[u];
                if (sp.target_id == v &&
                    operator_costs[op] == operator_costs[sp.op_id]) {
                    set_parent(u, Transition(op, state), backward);
                }
            }
        }
    }

    /*
      Instead of just recursively inserting all orphans, we first push them
      into a candidate queue that is sorted by (old, possibly too low)
      h-values. Then, we try to reconnect them to a non-orphaned state at
      no additional cost. Only if that fails, we flag the candidate as
      orphaned and push its SPT-children (who have strictly larger h-values
      due to no 0-cost operators) into the candidate queue.
    */

    /*
      If we split a state that's an ancestor of the initial state in the SPT,
      we know that exactly one of v1 or v2 is still settled. This allows us to
      push only one of them into the candidate queue. With splits that don't
      consider the SPT, we cannot make this optimization anymore and need to
      add both states to the candidate queue.
    */
    assert(all_of(states.begin(), states.end(), [backward](const StateInfo &s) {
                      return !s.dirty || s.init_distance == INF_COSTS || s.goal_distance == INF_COSTS;
                  }));

    // With conditional effects, several values in the same variable may be
    // necessary for a transition, so some transitions can be in none of the
    // children after the split.
    if (!backward && old_incoming.has_value()) {
        for (Transition t : old_incoming.value()) {
            if (use_cache) {
                for (const Transition &t : parents[t.target_id]) {
                    if (t.target_id == v && !states[t.target_id].dirty && !states[t.target_id].dirty_candidate) {
                        states[t.target_id].dirty_candidate = true;
                        candidate_queue.push(states[t.target_id].goal_distance, t.target_id);
                        break;
                    }
                }
            } else {
                if (parent[t.target_id].target_id == v && !states[t.target_id].dirty && !states[t.target_id].dirty_candidate) {
                    states[t.target_id].dirty_candidate = true;
                    candidate_queue.push(states[t.target_id].goal_distance, t.target_id);
                    if (debug) {
                        log << "Push to candidate queue: " << states[t.target_id].goal_distance << ", " << t.target_id << endl;
                    }
                }
            }
        }
    }
    if (backward && old_outgoing.has_value()) {
        for (Transition t : old_outgoing.value()) {
            if (use_cache) {
                for (const Transition &t : reverse_parents[t.target_id]) {
                    if (t.target_id == v && !states[t.target_id].dirty && !states[t.target_id].dirty_candidate) {
                        states[t.target_id].dirty_candidate = true;
                        candidate_queue.push(states[t.target_id].init_distance, t.target_id);
                        if (debug) {
                            log << "Push to candidate queue: " << states[t.target_id].init_distance << ", " << t.target_id << endl;
                        }
                        break;
                    }
                }
            } else {
                if (reverse_parent[t.target_id].target_id == v && !states[t.target_id].dirty && !states[t.target_id].dirty_candidate) {
                    states[t.target_id].dirty_candidate = true;
                    candidate_queue.push(states[t.target_id].init_distance, t.target_id);
                    if (debug) {
                        log << "Push to candidate queue: " << states[t.target_id].init_distance << ", " << t.target_id << endl;
                    }
                }
            }
        }
    }
    // They may have been marked as dirty in the other direction if they
    // are dead-ends.
    if (!states[v1].dirty) {
        states[v1].dirty_candidate = true;
        candidate_queue.push(states[v1].init_distance, v1);
        if (debug) {
            log << "Push to candidate queue: " << states[v1].init_distance << ", " << v1 << endl;
        }
    }
    if (!states[v2].dirty) {
        states[v2].dirty_candidate = true;
        candidate_queue.push(states[v2].init_distance, v2);
        if (debug) {
            log << "Push to candidate queue: " << states[v2].init_distance << ", " << v2 << endl;
        }
    }

    // So, after this all dirty states are marked.
    while (!candidate_queue.empty()) {
        int state = candidate_queue.pop().second;
        assert(states[state].dirty_candidate);
        if (debug) {
            log << "Try to reconnect " << state
                << " with h=" << (backward ? states[state].init_distance : states[state].goal_distance) << endl;
        }
        // If the distance is actually 0 (goal in forward direction and init
        // state in backward direction) the state must not be reconnected nor
        // marked as dirty.
        if (backward) {
            if (state == initial_state) {
                states[state].dirty_candidate = false;
                continue;
            }
        } else {
            if (goals.count(state)) {
                states[state].dirty_candidate = false;
                continue;
            }
        }
        assert(states[state].dirty_candidate);
        assert(backward ? states[state].init_distance != INF_COSTS : states[state].goal_distance != INF_COSTS);
        assert(!states[state].dirty);
        bool reconnected = false;
        // Try to reconnect to settled, solvable state.
        if (use_cache) {
            // Remove invalid transitions from children and parents vectors.
            if (backward) {
                int num_parents_before = reverse_parents[state].size();
                reverse_parents[state].erase(
                    remove_if(
                        reverse_parents[state].begin(), reverse_parents[state].end(),
                        [&](const Transition &reverse_parent) {
                            assert(abstraction.has_transition(reverse_parent.target_id, reverse_parent.op_id, state));
                            bool valid_parent = !states[reverse_parent.target_id].dirty;
                            if (!valid_parent) {
                                remove_child(reverse_parent.target_id, Transition(reverse_parent.op_id, state), true);
                            }
                            return !valid_parent;
                        }), reverse_parents[state].end());
                int num_parents_after = reverse_parents[state].size();
                num_cached_shortest_paths += num_parents_after - num_parents_before;
                reconnected = !reverse_parents[state].empty();
            } else {
                int num_parents_before = parents[state].size();
                parents[state].erase(
                    remove_if(
                        parents[state].begin(), parents[state].end(),
                        [&](const Transition &parent) {
                            assert(abstraction.has_transition(state, parent.op_id, parent.target_id));
                            bool valid_parent = !states[parent.target_id].dirty;
                            if (!valid_parent) {
                                remove_child(parent.target_id, Transition(parent.op_id, state));
                            }
                            return !valid_parent;
                        }), parents[state].end());
                int num_parents_after = parents[state].size();
                num_cached_shortest_paths += num_parents_after - num_parents_before;
                reconnected = !parents[state].empty();
            }
        } else {
            if (backward) {
                for (const Transition &t : abstraction.get_incoming_transitions(state)) {
                    int succ = t.target_id;
                    int op_id = t.op_id;
                    if (!states[succ].dirty &&
                        add_costs(states[succ].init_distance, operator_costs[op_id]) == states[state].init_distance) {
                        if (debug) {
                            log << "Reconnect " << state << " to " << succ << " via "
                                << op_id << " with cost " << operator_costs[op_id]
                                << " (" << convert_to_32_bit_cost(operator_costs[op_id])
                                << ")" << endl;
                        }
                        assert(states[state].init_distance != INF_COSTS);
                        assert(states[succ].init_distance != INF_COSTS);
                        assert(operator_costs[op_id] != INF_COSTS);
                        set_parent(state, Transition(op_id, succ), true);
                        reconnected = true;
                        break;
                    }
                }
            } else {
                for (const Transition &t : abstraction.get_outgoing_transitions(state)) {
                    int succ = t.target_id;
                    int op_id = t.op_id;
                    if (!states[succ].dirty &&
                        add_costs(states[succ].goal_distance, operator_costs[op_id]) == states[state].goal_distance) {
                        if (debug) {
                            log << "Reconnect " << state << " to " << succ << " via "
                                << op_id << " with cost " << operator_costs[op_id]
                                << " (" << convert_to_32_bit_cost(operator_costs[op_id])
                                << ")" << endl;
                        }
                        assert(states[state].goal_distance != INF_COSTS);
                        assert(states[succ].goal_distance != INF_COSTS);
                        assert(operator_costs[op_id] != INF_COSTS);
                        set_parent(state, Transition(op_id, succ));
                        reconnected = true;
                        break;
                    }
                }
            }
        }
        if (debug) {
            log << "Reconnected: " << boolalpha << reconnected << endl;
        }
        if (!reconnected) {
            mark_dirty(state, backward);

            if (use_cache) {
                if (g_hacked_sort_transitions) {
                    sort(execution::unseq, children[state].begin(), children[state].end());
                }
                if (backward) {
                    for (const Transition &t : reverse_children[state]) {
                        int prev = t.target_id;
                        if (!states[prev].dirty_candidate && !states[prev].dirty) {
                            if (debug) {
                                log << "Add " << prev << " to candidate queue" << endl;
                            }
                            states[prev].dirty_candidate = true;
                            candidate_queue.push(states[prev].init_distance, prev);
                        }
                    }
                } else {
                    for (const Transition &t : children[state]) {
                        int prev = t.target_id;
                        if (!states[prev].dirty_candidate && !states[prev].dirty) {
                            if (debug) {
                                log << "Add " << prev << " to candidate queue" << endl;
                            }
                            states[prev].dirty_candidate = true;
                            candidate_queue.push(states[prev].goal_distance, prev);
                        }
                    }
                }
            } else {
                if (backward) {
                    for (const Transition &t : abstraction.get_outgoing_transitions(state)) {
                        int prev = t.target_id;
                        if (!states[prev].dirty_candidate &&
                            !states[prev].dirty &&
                            reverse_parent[prev].target_id == state) {
                            if (debug) {
                                log << "Add " << prev << " to candidate queue" << endl;
                            }
                            states[prev].dirty_candidate = true;
                            candidate_queue.push(states[prev].init_distance, prev);
                        }
                    }
                } else {
                    for (const Transition &t : abstraction.get_incoming_transitions(state)) {
                        int prev = t.target_id;
                        if (!states[prev].dirty_candidate &&
                            !states[prev].dirty &&
                            parent[prev].target_id == state) {
                            if (debug) {
                                log << "Add " << prev << " to candidate queue" << endl;
                            }
                            states[prev].dirty_candidate = true;
                            candidate_queue.push(states[prev].goal_distance, prev);
                        }
                    }
                }
            }
        }
        states[state].dirty_candidate = false;

        if (timer.is_expired()) {
            // Up to here all goal distances are always lower bounds, so we can abort at any time.
            log << "Timer expired --> abort incremental search" << endl;
            return;
        }
    }

#ifndef NDEBUG
    /*
      We use dirty_states to efficiently loop over dirty states. Check that all
      solvable states marked as dirty are part of the vector.
      We don't explicitly reset dirty states.
    */
    int num_states = states.size();
    for (int i = 0; i < num_states; ++i) {
        if (states[i].dirty && states[i].init_distance != INF_COSTS && states[i].goal_distance != INF_COSTS) {
            assert(count(dirty_states.begin(), dirty_states.end(), i) == 1);
        }
    }
    // Goal states must never be dirty.
    if (backward) {
        assert(!count(dirty_states.begin(), dirty_states.end(), initial_state));
    } else {
        for (int goal : abstraction.get_goals()) {
            assert(!count(dirty_states.begin(), dirty_states.end(), goal));
        }
    }
#endif

    /*
      Perform a Dijkstra-style exploration to recompute all h values as
      follows. The "initial state" of the search is a virtual state that
      represents all settled states. It is expanded first, starting with a cost
      of 0. Its outgoing arcs are all arcs (in the backward graph) that go from
      a settled state s to a dirty state s' with operator o, and the cost of
      the transition is h(s) + cost(o). (Note that h(s) for settled states is
      known.) After this initialization, proceed with a normal Dijkstra search,
      but only consider arcs that lead from dirty to dirty states.
    */
    open_queue.clear();
    for (int state : dirty_states) {
        assert(states[state].dirty);
        Cost min_dist = INF_COSTS;
        if (debug) {
            log << "Dirty state: " << state << endl;
        }
        for (const Transition &t : backward ? abstraction.get_incoming_transitions(state) :
             abstraction.get_outgoing_transitions(state)) {
            int succ = t.target_id;
            int op_id = t.op_id;
            if (debug) {
                log << "Transition in op " << t.op_id << " with target " << t.target_id << endl;
            }
            if (!states[succ].dirty) {
                Cost succ_dist = backward ? states[succ].init_distance : states[succ].goal_distance;
                Cost cost = operator_costs[op_id];
                Cost new_dist = add_costs(cost, succ_dist);
                if (debug) {
                    log << "Cost: " << cost << " (" << convert_to_32_bit_cost(cost) << ")" << endl;
                    log << "Succ: " << succ << endl;
                    log << "succ_dist: " << succ_dist << " (" << convert_to_32_bit_cost(succ_dist) << ")" << endl;
                    log << "new_dist: " << new_dist << " (" << convert_to_32_bit_cost(new_dist) << ")" << endl;
                    log << "min_dist: " << min_dist << " (" << convert_to_32_bit_cost(min_dist) << ")" << endl;
                }
                if (new_dist < min_dist) {
                    min_dist = new_dist;
                    set_parent(state, Transition(op_id, succ), backward);
                } else if (use_cache && new_dist != INF_COSTS && new_dist == min_dist) {
                    add_parent(state, Transition(op_id, succ), backward);
                }
            }
        }
        if (backward) {
            states[state].init_distance = min_dist;
        } else {
            states[state].goal_distance = min_dist;
        }
        if (min_dist != INF_COSTS) {
            open_queue.push(min_dist, state);
            if (debug) {
                log << "Push to open_queue: (min_dist: " << min_dist << ", state: " << state << ")" << endl;
            }
        }
    }

    if (debug) {
        log << "Dirty states: ";
        for (int state : dirty_states) {
            if (states[state].dirty) {
                log << state << ", ";
            }
        }
        log << endl;
    }

    while (!open_queue.empty()) {
        pair<Cost, int> top_pair = open_queue.pop();
        const Cost g = top_pair.first;
        const int state = top_pair.second;
        assert(count(dirty_states.begin(), dirty_states.end(), state) == 1);
        if (g > (backward ? states[state].init_distance : states[state].goal_distance)) {
            if (debug) {
                log << "continue because g > dist, " << g << " > " << (backward ? states[state].init_distance :
                                                                       states[state].goal_distance) << endl;
            }
            continue;
        }
        assert(g == (backward ? states[state].init_distance : states[state].goal_distance));
        assert(g != INF_COSTS);
        assert(states[state].dirty);
        states[state].dirty = false;
        if (debug) {
            log << "state " << state << " cleaned" << endl;
        }
        for (const Transition &t : backward ? abstraction.get_outgoing_transitions(state) :
             abstraction.get_incoming_transitions(state)) {
            int succ = t.target_id;
            int op_id = t.op_id;
            Cost cost = operator_costs[op_id];
            Cost succ_g = add_costs(cost, g);

            if (states[succ].dirty &&
                succ_g < (backward ? states[succ].init_distance : states[succ].goal_distance)) {
                assert(count(dirty_states.begin(), dirty_states.end(), succ) == 1);
                if (backward) {
                    states[succ].init_distance = succ_g;
                } else {
                    states[succ].goal_distance = succ_g;
                }
                set_parent(succ, Transition(op_id, state), backward);
                open_queue.push(succ_g, succ);
                if (debug) {
                    log << "Push to open_queue: (succ_g: " << succ_g << ", succ: " << succ << ")" << endl;
                }
            } else if (use_cache && states[succ].dirty &&
                       succ_g == (backward ? states[succ].init_distance : states[succ].goal_distance) &&
                       succ_g != INF_COSTS) {
                add_parent(succ, Transition(op_id, state), backward);
            }
        }
    }
}

Cost ShortestPaths::get_64bit_goal_distance(int abstract_state_id) const {
    return states[abstract_state_id].goal_distance;
}

int ShortestPaths::get_32bit_goal_distance(int abstract_state_id) const {
    return convert_to_32_bit_cost(get_64bit_goal_distance(abstract_state_id));
}

bool ShortestPaths::is_optimal_transition(int start_id, int op_id, int target_id) const {
    return states[start_id].goal_distance - operator_costs[op_id] == states[target_id].goal_distance;
}
bool ShortestPaths::is_optimal_backward_transition(int start_id, int op_id, int target_id) const {
    return states[start_id].init_distance - operator_costs[op_id] == states[target_id].init_distance;
}

OptimalTransitions ShortestPaths::get_optimal_transitions(
    const Abstraction &abstraction, int state) const {
    OptimalTransitions transitions;
    if (use_cache) {
        for (const Transition &t : parents[state]) {
            transitions[t.op_id].push_back(t.target_id);
        }
        if (g_hacked_sort_transitions) {
            for (auto &[op_id, transitions_for_op]: transitions) {
                sort(execution::unseq, transitions_for_op.begin(), transitions_for_op.end());
            }
        }
    } else {
        for (const Transition &t : abstraction.get_outgoing_transitions(state)) {
            if (is_optimal_transition(state, t.op_id, t.target_id)) {
                transitions[t.op_id].push_back(t.target_id);
            }
        }
    }
    return transitions;
}

OptimalTransitions ShortestPaths::get_optimal_backward_transitions(
    const Abstraction &abstraction, int state) const {
    OptimalTransitions transitions;
    if (use_cache) {
        for (const Transition &t : reverse_parents[state]) {
            transitions[t.op_id].push_back(t.target_id);
        }
        if (g_hacked_sort_transitions) {
            for (auto &[op_id, transitions_for_op]: transitions) {
                sort(execution::unseq, transitions_for_op.begin(), transitions_for_op.end());
            }
        }
    } else {
        for (const Transition &t : abstraction.get_incoming_transitions(state)) {
            if (is_optimal_backward_transition(state, t.op_id, t.target_id)) {
                transitions[t.op_id].push_back(t.target_id);
            }
        }
    }
    return transitions;
}

#ifndef NDEBUG
bool ShortestPaths::test_distances(
    const Abstraction &abstraction,
    const Goals &goals) {
    assert(all_of(states.begin(), states.end(), [](const StateInfo &s) {
                      return !s.dirty || s.goal_distance == INF_COSTS || s.init_distance == INF_COSTS;
                  }));
    int num_states = abstraction.get_num_states();

    vector<int> costs;
    costs.reserve(operator_costs.size());
    for (Cost cost : operator_costs) {
        costs.push_back(convert_to_32_bit_cost(cost));
    }

    int init_state = abstraction.get_initial_state_id();
    vector<int> computed_init_distances = compute_init_distances(abstraction, costs, init_state);

    for (int v = 0; v < num_states; ++v) {
        if (debug) {
            log << "Test state " << v << endl;
        }
        if (use_cache) {
            if (debug) {
                log << "parents: " << parents[v] << endl;
                log << "children: " << children[v] << endl;
                log << "reverse_parents: " << reverse_parents[v] << endl;
                log << "reverse_children: " << reverse_children[v] << endl;
            }
            for (const Transition &parent : parents[v]) {
                int w = parent.target_id;
                int op_id = parent.op_id;
                assert(count(children[w].begin(), children[w].end(), Transition(op_id, v)) == 1);
                assert(abstraction.has_transition(v, op_id, w));
            }
            for (const Transition &child : children[v]) {
                int u = child.target_id;
                int op_id = child.op_id;
                assert(count(parents[u].begin(), parents[u].end(), Transition(op_id, v)) == 1);
                assert(abstraction.has_transition(u, op_id, v));
            }
            for (const Transition &parent : reverse_parents[v]) {
                int w = parent.target_id;
                int op_id = parent.op_id;
                assert(count(reverse_children[w].begin(), reverse_children[w].end(), Transition(op_id, v)) == 1);
                assert(abstraction.has_transition(w, op_id, v));
            }
            for (const Transition &child : reverse_children[v]) {
                int u = child.target_id;
                int op_id = child.op_id;
                assert(count(reverse_parents[u].begin(), reverse_parents[u].end(), Transition(op_id, v)) == 1);
                assert(abstraction.has_transition(v, op_id, u));
            }
        } else {
            if (states[v].goal_distance == INF_COSTS ||
                states[v].init_distance == INF_COSTS) {
                continue;
            }
            const Transition &t = parent[v];
            const Transition &ct = reverse_parent[v];
            if (debug) {
                log << "Parent: " << t << endl;
                log << "Child: " << ct << endl;
            }
            Transitions out = abstraction.get_outgoing_transitions(v);
            Transitions in = abstraction.get_incoming_transitions(v);
            if (debug) {
                log << "Outgoing transitions: " << out << endl;
                log << "Incoming transitions: " << in << endl;
                if (!goals.count(v)) {
                    assert(t.is_defined());
                    assert(count(out.begin(), out.end(), t) == 1);
                    assert(states[v].goal_distance ==
                           add_costs(operator_costs[t.op_id], states[t.target_id].goal_distance));
                }
                if (v != init_state && states[v].init_distance != INF_COSTS) {
                    assert(ct.is_defined());
                    assert(count(in.begin(), in.end(), ct) == 1);
                    assert(states[v].init_distance ==
                           add_costs(operator_costs[ct.op_id], states[ct.target_id].init_distance));
                }
            }
        }
    }

    vector<int> goal_distances_32_bit = compute_goal_distances(abstraction, costs, goals);
    vector<int> goal_distances_32_bit_rounded_down = get_goal_distances();

    for (int i = 0; i < num_states; ++i) {
        if (!states[i].dirty) {
            if ((goal_distances_32_bit_rounded_down[i] != goal_distances_32_bit[i] ||
                 convert_to_32_bit_cost(states[i].init_distance) != computed_init_distances[i]) &&
                computed_init_distances[i] != INF) {
                log << "32-bit INF: " << INF << endl;
                log << "64-bit 0: " << convert_to_64_bit_cost(0) << endl;
                log << "64-bit 1: " << convert_to_64_bit_cost(1) << endl;
                log << "64-bit INF: " << INF_COSTS << endl;
                log << "32-bit rounded:   " << goal_distances_32_bit_rounded_down << endl;
                log << "32-bit distances: " << goal_distances_32_bit << endl;
                log << "state: " << i << endl;
                log << "init_distance: " << states[i].init_distance << endl;
                log << "computed_init_distance: " << computed_init_distances[i] << endl;

                assert(convert_to_32_bit_cost(states[i].init_distance) == computed_init_distances[i]);

                ABORT("Distances are wrong.");
            }
            assert(convert_to_32_bit_cost(states[i].init_distance) == computed_init_distances[i]);
        }
    }

    if (use_cache) {
        int real_num_parents = 0;
        for (const auto &p : parents) {
            real_num_parents += p.size();
        }
        int real_num_children = 0;
        for (const auto &p : children) {
            real_num_children += p.size();
        }
        int real_num_reverse_parents = 0;
        for (const auto &p : reverse_parents) {
            real_num_reverse_parents += p.size();
        }
        int real_num_reverse_children = 0;
        for (const auto &p : reverse_children) {
            real_num_reverse_children += p.size();
        }
        if (debug) {
            log << "num_cached_shortest_paths: " << num_cached_shortest_paths << endl;
            log << "real_num_parents: " << real_num_parents << endl;
            log << "real_num_children: " << real_num_children << endl;
            log << "real_num_reverse_parents: " << real_num_reverse_parents << endl;
            log << "real_num_reverse_children: " << real_num_reverse_children << endl;
        }
        assert(num_cached_shortest_paths == real_num_parents + real_num_reverse_parents);
    }

    return true;
}
#endif

void ShortestPaths::print_statistics() const {
    if (log.is_at_least_verbose()) {
        map<int, int> children_counts;
        for (const auto &c : children) {
            children_counts[c.size()] += 1;
        }
        log << "SPT children: " << children_counts << endl;
        map<int, int> parents_counts;
        for (const auto &p : parents) {
            parents_counts[p.size()] += 1;
        }
        log << "SPT parents: " << parents_counts << endl;

        map<int, int> reverse_children_counts;
        for (const auto &c : reverse_children) {
            reverse_children_counts[c.size()] += 1;
        }
        log << "SPT reverse_children: " << reverse_children_counts << endl;
        map<int, int> reverse_parents_counts;
        for (const auto &p : reverse_parents) {
            reverse_parents_counts[p.size()] += 1;
        }
        log << "SPT reverse_parents: " << reverse_parents_counts << endl;

        log << "SPT stored transitions: " << num_cached_shortest_paths << endl;
    }
}

vector<int> compute_goal_distances(
    const Abstraction &abstraction,
    const vector<int> &costs,
    const unordered_set<int> &goal_ids) {
    vector<int> distances(abstraction.get_num_states(), INF);
    priority_queues::AdaptiveQueue<int> open_queue;
    for (int goal_id : goal_ids) {
        distances[goal_id] = 0;
        open_queue.push(0, goal_id);
    }
    while (!open_queue.empty()) {
        pair<int, int> top_pair = open_queue.pop();
        int old_g = top_pair.first;
        int state_id = top_pair.second;

        const int g = distances[state_id];
        assert(0 <= g && g < INF);
        assert(g <= old_g);
        if (g < old_g)
            continue;
        for (const Transition &transition : abstraction.get_incoming_transitions(state_id)) {
            const int op_cost = costs[transition.op_id];
            assert(op_cost >= 0);
            int succ_g = (op_cost == INF) ? INF : g + op_cost;
            assert(succ_g >= 0);
            int succ_id = transition.target_id;
            if (succ_g < distances[succ_id]) {
                distances[succ_id] = succ_g;
                open_queue.push(succ_g, succ_id);
            }
        }
    }
    return distances;
}
vector<int> compute_init_distances(
    const Abstraction &abstraction,
    const vector<int> &costs,
    const int init_id) {
    vector<int> distances(abstraction.get_num_states(), INF);
    priority_queues::AdaptiveQueue<int> open_queue;
    distances[init_id] = 0;
    open_queue.push(0, init_id);
    while (!open_queue.empty()) {
        pair<int, int> top_pair = open_queue.pop();
        int old_g = top_pair.first;
        int state_id = top_pair.second;

        const int g = distances[state_id];
        assert(0 <= g && g < INF);
        assert(g <= old_g);
        if (g < old_g)
            continue;
        for (const Transition &transition : abstraction.get_outgoing_transitions(state_id)) {
            const int op_cost = costs[transition.op_id];
            assert(op_cost >= 0);
            int succ_g = (op_cost == INF) ? INF : g + op_cost;
            assert(succ_g >= 0);
            int succ_id = transition.target_id;
            if (succ_g < distances[succ_id]) {
                distances[succ_id] = succ_g;
                open_queue.push(succ_g, succ_id);
            }
        }
    }
    return distances;
}
}
