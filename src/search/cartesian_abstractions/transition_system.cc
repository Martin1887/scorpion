#include "transition_system.h"

#include "abstract_state.h"
#include "transition.h"
#include "utils.h"

#include "../task_proxy.h"

#include "../task_utils/disambiguated_operator.h"
#include "../task_utils/task_properties.h"
#include "../utils/logging.h"

#include <algorithm>
#include <map>

using namespace std;
using namespace disambiguation;

namespace cartesian_abstractions {
static void remove_transitions_with_given_target(
    Transitions &transitions, int state_id) {
    auto new_end = remove_if(
        transitions.begin(), transitions.end(),
        [state_id](const Transition &t) {return t.target_id == state_id;});
    assert(new_end != transitions.end());
    transitions.erase(new_end, transitions.end());
}


TransitionSystem::TransitionSystem(const shared_ptr<vector<DisambiguatedOperator>> &ops)
    : operators(ops),
      num_non_loops(0),
      num_loops(0) {
}

void TransitionSystem::enlarge_vectors_by_one() {
    int new_num_states = get_num_states() + 1;
    outgoing.resize(new_num_states);
    incoming.resize(new_num_states);
    loops.resize(new_num_states);
}

void TransitionSystem::add_loops_in_trivial_abstraction(const AbstractState &init,
                                                        const bool disambiguated) {
    assert(get_num_states() == 0);
    enlarge_vectors_by_one();
    int init_id = 0;
    for (const disambiguation::DisambiguatedOperator &op : *operators) {
        // The initial abstract state could be disambiguated.
        if (!op.is_redundant() && (!disambiguated ||
                                   (init.is_applicable(op) &&
                                    init.reach_with_op(init, op)))) {
            add_loop(init_id, op.get_id());
        }
    }
}

void TransitionSystem::add_transition(int src_id, int op_id, int target_id) {
    assert(src_id != target_id);
    outgoing[src_id].emplace_back(op_id, target_id);
    incoming[target_id].emplace_back(op_id, src_id);
    ++num_non_loops;
}

void TransitionSystem::add_loop(int state_id, int op_id) {
    assert(utils::in_bounds(state_id, loops));
    loops[state_id].push_back(op_id);
    ++num_loops;
}

void TransitionSystem::force_new_transitions(const std::vector<Transitions> &new_incoming,
                                             const std::vector<Transitions> &new_outgoing,
                                             const std::vector<Loops> &new_loops) {
    incoming = new_incoming;
    outgoing = new_outgoing;
    loops = new_loops;
}

void TransitionSystem::rewire_incoming_transitions(
    const Transitions &old_incoming, const AbstractStates &states, int v_id,
    const AbstractState &v1, const AbstractState &v2,
    const vector<int> &modified_vars) {
    /* State v has been split into v1 and v2. Now for all transitions
       u->v we need to add transitions u->v1, u->v2, or both. */
    int v1_id = v1.get_id();
    int v2_id = v2.get_id();

    unordered_set<int> updated_states;
    for (const Transition &transition : old_incoming) {
        int u_id = transition.target_id;
        bool is_new_state = updated_states.insert(u_id).second;
        if (is_new_state) {
            remove_transitions_with_given_target(outgoing[u_id], v_id);
        }
    }
    num_non_loops -= old_incoming.size();

    for (const Transition &transition : old_incoming) {
        int op_id = transition.op_id;
        int u_id = transition.target_id;
        const AbstractState &u = *states[u_id];
        if (u.reach_with_op(v1, (*operators)[op_id], modified_vars)) {
            add_transition(u_id, op_id, v1_id);
        }
        if (u.reach_with_op(v2, (*operators)[op_id], modified_vars)) {
            add_transition(u_id, op_id, v2_id);
        }
    }
}

void TransitionSystem::rewire_outgoing_transitions(
    const Transitions &old_outgoing, const AbstractStates &states, int v_id,
    const AbstractState &v1, const AbstractState &v2,
    const vector<int> &modified_vars) {
    /* State v has been split into v1 and v2. Now for all transitions
       v->w we need to add transitions v1->w, v2->w, or both. */
    int v1_id = v1.get_id();
    int v2_id = v2.get_id();

    unordered_set<int> updated_states;
    for (const Transition &transition : old_outgoing) {
        int w_id = transition.target_id;
        bool is_new_state = updated_states.insert(w_id).second;
        if (is_new_state) {
            remove_transitions_with_given_target(incoming[w_id], v_id);
        }
    }
    num_non_loops -= old_outgoing.size();

    for (const Transition &transition : old_outgoing) {
        int op_id = transition.op_id;
        int w_id = transition.target_id;
        const AbstractState &w = *states[w_id];
        if (v1.is_applicable((*operators)[op_id], modified_vars) &&
            v1.reach_with_op(w, (*operators)[op_id], modified_vars)) {
            add_transition(v1_id, op_id, w_id);
        }
        if (v2.is_applicable((*operators)[op_id], modified_vars) &&
            v2.reach_with_op(w, (*operators)[op_id], modified_vars)) {
            add_transition(v2_id, op_id, w_id);
        }
    }
}

void TransitionSystem::rewire_loops(
    const Loops &old_loops, const AbstractState &v1, const AbstractState &v2,
    const vector<int> &modified_vars,
    const bool simulated) {
    /* State v has been split into v1 and v2. Now for all self-loops
       v->v we need to add one or two of the transitions v1->v1, v1->v2,
       v2->v1 and v2->v2. */
    int v1_id = v1.get_id();
    int v2_id = v2.get_id();
    for (int op_id : old_loops) {
        bool applicable_v1 = v1.is_applicable((*operators)[op_id], modified_vars);
        bool applicable_v2 = v2.is_applicable((*operators)[op_id], modified_vars);
        bool reach_v1_from_v1 = v1.reach_with_op(v1, (*operators)[op_id], modified_vars);
        bool reach_v2_from_v1 = v1.reach_with_op(v2, (*operators)[op_id], modified_vars);
        bool reach_v1_from_v2 = v2.reach_with_op(v1, (*operators)[op_id], modified_vars);
        bool reach_v2_from_v2 = v2.reach_with_op(v2, (*operators)[op_id], modified_vars);
        if (!simulated) {
            if (reach_v1_from_v1 && applicable_v1) {
                add_loop(v1_id, op_id);
            }
            if (reach_v2_from_v2 && applicable_v2) {
                add_loop(v2_id, op_id);
            }
        }
        if (reach_v2_from_v1 && applicable_v1) {
            add_transition(v1_id, op_id, v2_id);
        }
        if (reach_v1_from_v2 && applicable_v2) {
            add_transition(v2_id, op_id, v1_id);
        }
    }
    num_loops -= old_loops.size();
}

tuple<Transitions, Transitions> TransitionSystem::rewire(
    const AbstractStates &states, int v_id,
    const AbstractState &v1, const AbstractState &v2,
    const bool simulated) {
    // Retrieve old transitions and make space for new transitions.
    Transitions old_incoming = move(incoming[v_id]);
    Transitions old_outgoing = move(outgoing[v_id]);
    Loops old_loops = move(loops[v_id]);
    enlarge_vectors_by_one();
    int v1_id = v1.get_id();
    int v2_id = v2.get_id();
    utils::unused_variable(v1_id);
    utils::unused_variable(v2_id);
    assert(incoming[v1_id].empty() && outgoing[v1_id].empty() && loops[v1_id].empty());
    assert(incoming[v2_id].empty() && outgoing[v2_id].empty() && loops[v2_id].empty());

    vector<int> modified_vars{};
    const CartesianSet &v_set = states[v_id]->get_cartesian_set();
    const CartesianSet &v1_set = v1.get_cartesian_set();
    const CartesianSet &v2_set = v2.get_cartesian_set();
    int n_vars = v_set.get_n_vars();
    for (int var = 0; var < n_vars; var++) {
        if (!v_set.is_equal_in_var(v1_set, var) || !v_set.is_equal_in_var(v2_set, var)) {
            modified_vars.push_back(var);
        }
    }

    // Remove old transitions and add new transitions.
    rewire_incoming_transitions(old_incoming, states, v_id, v1, v2, modified_vars);
    rewire_outgoing_transitions(old_outgoing, states, v_id, v1, v2, modified_vars);
    // For a simulated rewire, loops can be omitted because they will not be
    // used in future iterations.
    rewire_loops(old_loops, v1, v2, modified_vars, simulated);

    return {old_incoming, old_outgoing};
}

const vector<Transitions> &TransitionSystem::get_incoming_transitions() const {
    return incoming;
}

const vector<Transitions> &TransitionSystem::get_outgoing_transitions() const {
    return outgoing;
}

const vector<Loops> &TransitionSystem::get_loops() const {
    return loops;
}

const CartesianState &TransitionSystem::get_preconditions(int op_id) const {
    return (*operators)[op_id].get_precondition();
}

int TransitionSystem::get_num_states() const {
    assert(incoming.size() == outgoing.size());
    assert(loops.size() == outgoing.size());
    return outgoing.size();
}

int TransitionSystem::get_num_operators() const {
    return operators->size();
}

int TransitionSystem::get_num_non_loops() const {
    return num_non_loops;
}

int TransitionSystem::get_num_loops() const {
    return num_loops;
}

const shared_ptr<vector<DisambiguatedOperator>> &TransitionSystem::get_operators() const {
    return operators;
}

void TransitionSystem::print_statistics(utils::LogProxy &log) const {
    if (log.is_at_least_normal()) {
        int total_incoming_transitions = 0;
        utils::unused_variable(total_incoming_transitions);
        int total_outgoing_transitions = 0;
        int total_loops = 0;
        for (int state_id = 0; state_id < get_num_states(); ++state_id) {
            total_incoming_transitions += incoming[state_id].size();
            total_outgoing_transitions += outgoing[state_id].size();
            total_loops += loops[state_id].size();
        }
        assert(total_outgoing_transitions == total_incoming_transitions);
        assert(get_num_loops() == total_loops);
        assert(get_num_non_loops() == total_outgoing_transitions);
        log << "Looping transitions: " << total_loops << endl;
        log << "Non-looping transitions: " << total_outgoing_transitions << endl;
    }
}

void TransitionSystem::dump() const {
    for (int i = 0; i < get_num_states(); ++i) {
        cout << "State " << i << endl;
        cout << "  in: " << incoming[i] << endl;
        cout << "  out: " << outgoing[i] << endl;
        cout << "  loops: " << loops[i] << endl;
    }
}
}
