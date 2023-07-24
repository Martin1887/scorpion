#include "shortest_paths.h"

#include "abstract_search.h"  // For test_distances().
#include "utils.h"

#include "../utils/logging.h"
#include "../utils/memory.h"

using namespace std;

namespace cegar {
const Cost ShortestPaths::DIRTY = numeric_limits<Cost>::max() - 1;

ShortestPaths::ShortestPaths(const vector<int> &costs, utils::LogProxy &log)
    : log(log),
      debug(log.is_at_least_debug()),
      task_has_zero_costs(any_of(costs.begin(), costs.end(), [](int c) {return c == 0;})) {
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
    assert(a != DIRTY && b != DIRTY);
    return (a == INF_COSTS || b == INF_COSTS) ? INF_COSTS : a + b;
}

int ShortestPaths::convert_to_32_bit_cost(Cost cost) const {
    assert(cost != DIRTY);
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

void ShortestPaths::recompute(
    const vector<Transitions> &in,
    const vector<Transitions> &out,
    const unordered_set<int> &goals,
    const int initial_state) {
    shortest_path = Transitions(in.size());
    reverse_shortest_path = Transitions(in.size());
    goal_distances = vector<Cost>(in.size(), INF_COSTS);
    init_distances = vector<Cost>(in.size(), INF_COSTS);
    open_queue.clear();
    recompute_forward(in, goals);
    open_queue.clear();
    recompute_backward(out, initial_state);
}

void ShortestPaths::recompute_forward(
    const vector<Transitions> &in,
    const unordered_set<int> &goals) {
    for (int goal : goals) {
        Cost dist = 0;
        goal_distances[goal] = dist;
        shortest_path[goal] = Transition();
        open_queue.push(dist, goal);
    }
    while (!open_queue.empty()) {
        pair<Cost, int> top_pair = open_queue.pop();
        Cost old_dist = top_pair.first;
        int state_id = top_pair.second;

        Cost dist = goal_distances[state_id];
        assert(dist < INF_COSTS);
        assert(dist <= old_dist);
        if (dist < old_dist)
            continue;
        assert(utils::in_bounds(state_id, in));
        for (const Transition &t : in[state_id]) {
            int succ_id = t.target_id;
            int op_id = t.op_id;
            Cost op_cost = operator_costs[op_id];
            Cost succ_dist = add_costs(dist, op_cost);
            if (succ_dist < goal_distances[succ_id]) {
                goal_distances[succ_id] = succ_dist;
                shortest_path[succ_id] = Transition(op_id, state_id);
                open_queue.push(succ_dist, succ_id);
            }
        }
    }
}

void ShortestPaths::recompute_backward(
    const vector<Transitions> &out,
    const int initial_state) {
    Cost dist = 0;
    init_distances[initial_state] = dist;
    reverse_shortest_path[initial_state] = Transition();
    open_queue.push(dist, initial_state);

    while (!open_queue.empty()) {
        pair<Cost, int> top_pair = open_queue.pop();
        Cost old_dist = top_pair.first;
        int state_id = top_pair.second;

        Cost dist = init_distances[state_id];
        assert(dist < INF_COSTS);
        assert(dist <= old_dist);
        if (dist < old_dist)
            continue;
        assert(utils::in_bounds(state_id, in));
        for (const Transition &t : out[state_id]) {
            int succ_id = t.target_id;
            int op_id = t.op_id;
            Cost op_cost = operator_costs[op_id];
            Cost succ_dist = add_costs(dist, op_cost);
            if (succ_dist < init_distances[succ_id]) {
                init_distances[succ_id] = succ_dist;
                reverse_shortest_path[succ_id] = Transition(op_id, state_id);
                open_queue.push(succ_dist, succ_id);
            }
        }
    }
}

void ShortestPaths::mark_dirty(int state, bool backward) {
    if (debug) {
        log << "Mark " << state << " as dirty" << endl;
    }
    if (backward) {
        init_distances[state] = DIRTY;
        // Previous shortest path is invalid now.
        reverse_shortest_path[state] = Transition();
    } else {
        goal_distances[state] = DIRTY;
        // Previous shortest path is invalid now.
        shortest_path[state] = Transition();
    }
    assert(!count(dirty_states.begin(), dirty_states.end(), state));
    dirty_states.push_back(state);
}

void ShortestPaths::update_incrementally(
    const vector<Transitions> &in,
    const vector<Transitions> &out,
    int v, int v1, int v2,
    const std::unordered_set<int> &goals,
    const int initial_state) {
    assert(in.size() == out.size());
    int num_states = in.size();

    shortest_path.resize(num_states);
    reverse_shortest_path.resize(num_states);
    goal_distances.resize(num_states, 0);
    init_distances.resize(num_states, 0);

    dirty_candidate.resize(num_states, false);
    dirty_states.clear();
    update_incrementally_in_direction(in, out, v, v1, v2, goals, initial_state, false);
    dirty_candidate.resize(num_states, false);
    dirty_states.clear();
    update_incrementally_in_direction(in, out, v, v1, v2, goals, initial_state, true);
}

void ShortestPaths::update_incrementally_in_direction(
    const std::vector<Transitions> &in,
    const std::vector<Transitions> &out,
    int v, int v1, int v2,
    const std::unordered_set<int> &goals,
    const int initial_state,
    bool backward) {
    vector<Cost> *distances = &goal_distances;
    Transitions *virtual_shortest_path = &shortest_path;
    const vector<Transitions> *virtual_in = &in;
    const vector<Transitions> *virtual_out = &out;
    string target_dist = "Goal ";
    if (backward) {
        distances = &init_distances;
        virtual_shortest_path = &reverse_shortest_path;
        virtual_in = &out;
        virtual_out = &in;
        target_dist = "Init ";
    }

    if (debug) {
        log << "Reflect splitting " << v << " into " << v1 << " and " << v2;
        if (backward) {
            log << " in backward direction";
        }
        log << endl;
        log << "Goal distances: " << goal_distances << endl;
        log << "Init distances: " << init_distances << endl;
        log << "Shortest paths: " << shortest_path << endl;
        log << "Reverse shortest paths: " << reverse_shortest_path << endl;
        log << "Goals: " << endl;
        for (auto goal : goals) {
            log << goal << endl;
        }
    }

    // Copy distance from split state. Distances will be updated if necessary.
    (*distances)[v1] = (*distances)[v2] = (*distances)[v];

    /* Update shortest path tree (SPT) transitions to v. The SPT transitions
       will be updated again if v1 or v2 are dirty. */
    for (int state : {v1, v2}) {
        for (const Transition &incoming : (*virtual_in)[state]) {
            int u = incoming.target_id;
            int op = incoming.op_id;
            Transition &sp = (*virtual_shortest_path)[u];
            if (sp.target_id == v &&
                operator_costs[op] == operator_costs[sp.op_id]) {
                sp = Transition(op, state);
            }
        }
    }

    if (debug) {
        log << "Goal distances: " << goal_distances << endl;
        log << "Init distances: " << init_distances << endl;
        log << "Shortest paths: " << shortest_path << endl;
        log << "Reverse shortest paths: " << reverse_shortest_path << endl;
    }

    /*
      Instead of just recursively inserting all orphans, we first push them
      into a candidate queue that is sorted by (old, possibly too low)
      h-values. Then, we try to reconnect them to a non-orphaned state at
      no additional cost. Only if that fails, we flag the candidate as
      orphaned and push its SPT-children (who have strictly larger h-values
      due to no 0-cost operators) into the candidate queue.
    */
    assert(candidate_queue.empty());
    assert(!count(dirty_candidate.begin(), dirty_candidate.end(), true));

    /*
      If we split a state that's an ancestor of the initial state in the SPT,
      we know that exactly one of v1 or v2 is still settled. This allows us to
      push only one of them into the candidate queue. With splits that don't
      consider the SPT, we cannot make this optimization anymore and need to
      add both states to the candidate queue.
    */
    dirty_candidate[v1] = true;
    dirty_candidate[v2] = true;
    candidate_queue.push(goal_distances[v1], v1);
    candidate_queue.push(goal_distances[v2], v2);

    // So, after this all dirty states are marked.
    while (!candidate_queue.empty()) {
        int state = candidate_queue.pop().second;
        // If the distance is actually 0 (goal in forward direction and init
        // state in backward direction) the state must not be reconnected nor
        // marked as dirty.
        if (backward) {
            if (state == initial_state) {
                dirty_candidate[state] = false;
                continue;
            }
        } else {
            if (goals.count(state)) {
                dirty_candidate[state] = false;
                continue;
            }
        }
        assert(dirty_candidate[state]);
        assert((*distances)[state] != INF_COSTS);
        assert((*distances)[state] != DIRTY);
        bool reconnected = false;
        // Try to reconnect to settled, solvable state.
        for (const Transition &t : (*virtual_out)[state]) {
            int succ = t.target_id;
            int op_id = t.op_id;
            if ((*distances)[succ] != DIRTY &&
                add_costs((*distances)[succ], operator_costs[op_id])
                == (*distances)[state]) {
                (*virtual_shortest_path)[state] = Transition(op_id, succ);
                reconnected = true;
                break;
            }
        }
        if (!reconnected) {
            mark_dirty(state, backward);
            for (const Transition &t : (*virtual_in)[state]) {
                int prev = t.target_id;
                if (!dirty_candidate[prev] &&
                    (*distances)[prev] != DIRTY &&
                    (*virtual_shortest_path)[prev].target_id == state) {
                    dirty_candidate[prev] = true;
                    candidate_queue.push((*distances)[prev], prev);
                }
            }
        }
        dirty_candidate[state] = false;
    }


    if (debug) {
        log << "Goal distances: " << goal_distances << endl;
        log << "Init distances: " << init_distances << endl;
        log << "Dirty states: " << dirty_states << endl;
    }

#ifndef NDEBUG
    /* We use dirty_states to efficiently loop over dirty states. Check that
       its data is consistent with the data in distances. */
    int num_states = in.size();
    vector<bool> dirty1(num_states, false);
    for (int state : dirty_states) {
        dirty1[state] = true;
    }

    vector<bool> dirty2(num_states, false);
    for (int state = 0; state < num_states; ++state) {
        if ((*distances)[state] == DIRTY) {
            dirty2[state] = true;
        }
    }
    assert(dirty1 == dirty2);
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
    int num_orphans = 0;
    open_queue.clear();
    for (int state : dirty_states) {
        Cost &dist = (*distances)[state];
        assert(dist == DIRTY);
        Cost min_dist = INF_COSTS;
        for (const Transition &t : (*virtual_out)[state]) {
            int succ = t.target_id;
            int op_id = t.op_id;
            if ((*distances)[succ] != DIRTY) {
                Cost succ_dist = (*distances)[succ];
                Cost cost = operator_costs[op_id];
                Cost new_dist = add_costs(cost, succ_dist);
                if (new_dist < min_dist) {
                    min_dist = new_dist;
                    (*virtual_shortest_path)[state] = Transition(op_id, succ);
                }
            }
        }
        dist = min_dist;
        if (min_dist != INF_COSTS) {
            open_queue.push(dist, state);
            ++num_orphans;
        }
    }
    while (!open_queue.empty()) {
        pair<Cost, int> top_pair = open_queue.pop();
        const Cost g = top_pair.first;
        const int state = top_pair.second;
        assert((*distances)[state] != DIRTY);
        if (g > (*distances)[state])
            continue;
        assert(g == (*distances)[state]);
        assert(g != INF_COSTS);
        for (const Transition &t : (*virtual_in)[state]) {
            int succ = t.target_id;
            int op_id = t.op_id;
            Cost cost = operator_costs[op_id];
            Cost succ_g = add_costs(cost, g);

            if ((*distances)[succ] == DIRTY || succ_g < (*distances)[succ]) {
                (*distances)[succ] = succ_g;
                (*virtual_shortest_path)[succ] = Transition(op_id, state);
                open_queue.push(succ_g, succ);
            }
        }
    }
}

unique_ptr<Solution> ShortestPaths::extract_solution(
    int init_id, const Goals &goals) {
    // h* = \infty iff goal is unreachable from this state.
    if (goal_distances[init_id] == INF_COSTS) {
        return nullptr;
    }

    int current_state = init_id;
    unique_ptr<Solution> solution = utils::make_unique_ptr<Solution>();
    // This happens at the beginning (when only the trivial state exists).
    // assert(!goals.count(current_state));
    while (!goals.count(current_state)) {
        assert(utils::in_bounds(current_state, shortest_path));
        const Transition &t = shortest_path[current_state];
        assert(t.op_id != UNDEFINED);
        assert(t.target_id != UNDEFINED);
        assert(t.target_id != current_state);
        assert(goal_distances[t.target_id] <= goal_distances[current_state]);
        solution->push_back(t);
        current_state = t.target_id;
    }
    return solution;
}

Cost ShortestPaths::get_64bit_goal_distance(int abstract_state_id) const {
    return goal_distances.at(abstract_state_id);
}

int ShortestPaths::get_32bit_goal_distance(int abstract_state_id) const {
    return convert_to_32_bit_cost(goal_distances.at(abstract_state_id));
}

bool ShortestPaths::is_optimal_transition(int start_id, int op_id, int target_id) const {
    return goal_distances[start_id] - operator_costs[op_id] == goal_distances[target_id];
}

bool ShortestPaths::is_backward_optimal_transition(int start_id, int op_id, int target_id) const {
        if (log.is_at_least_debug()) {
            log << "init_distances[start_id]: " << init_distances[start_id]
                << " - operator_costs[op_id]: " << operator_costs[op_id]
                << " == init_distances[target_id]?: " << init_distances[target_id] << endl;
        }
    return init_distances[start_id] - operator_costs[op_id] == init_distances[target_id];
}

bool ShortestPaths::test_distances(
    const vector<Transitions> &in,
    const vector<Transitions> &out,
    const unordered_set<int> &goals) {
    assert(none_of(goal_distances.begin(), goal_distances.end(),
                   [](Cost d) {return d == DIRTY;}));
    int num_states = in.size();

    vector<int> costs;
    costs.reserve(operator_costs.size());
    for (Cost cost : operator_costs) {
        costs.push_back(convert_to_32_bit_cost(cost));
    }

    int init_state = 0;
    vector<int> init_distances = compute_distances(out, costs, {init_state});

    for (int i = 0; i < num_states; ++i) {
        if (debug) {
            log << "Test state " << i << endl;
        }
        if (goal_distances[i] != INF_COSTS &&
            init_distances[i] != INF &&
            !goals.count(i)) {
            Transition t = shortest_path[i];
            if (debug) {
                log << "Shortest path: " << t << endl;
            }
            assert(t.is_defined());
            if (debug) {
                log << "Outgoing transitions: " << out[i] << endl;
            }
            assert(count(out[i].begin(), out[i].end(), t) == 1);
            assert(goal_distances[i] ==
                   add_costs(operator_costs[t.op_id], goal_distances[t.target_id]));
        }
    }

    vector<int> goal_distances_32_bit = compute_distances(in, costs, goals);
    vector<int> goal_distances_32_bit_rounded_down;
    goal_distances_32_bit_rounded_down.reserve(goal_distances_32_bit.size());
    for (Cost dist : goal_distances) {
        goal_distances_32_bit_rounded_down.push_back(convert_to_32_bit_cost(dist));
    }

    for (int i = 0; i < num_states; ++i) {
        if (goal_distances_32_bit_rounded_down[i] != goal_distances_32_bit[i] &&
            init_distances[i] != INF) {
            log << "32-bit INF: " << INF << endl;
            log << "64-bit 0: " << convert_to_64_bit_cost(0) << endl;
            log << "64-bit 1: " << convert_to_64_bit_cost(1) << endl;
            log << "64-bit INF: " << INF_COSTS << endl;
            log << "64-bit distances: " << goal_distances << endl;
            log << "32-bit rounded:   " << goal_distances_32_bit_rounded_down << endl;
            log << "32-bit distances: " << goal_distances_32_bit << endl;
            ABORT("Distances are wrong.");
        }
    }

    return true;
}
}
