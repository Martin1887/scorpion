#include "transition_system.h"

#include "abstract_state.h"
#include "transition.h"
#include "types.h"
#include "utils.h"

#include "../task_proxy.h"

#include "../task_utils/task_properties.h"
#include "../utils/logging.h"

#include <algorithm>
#include <map>

using namespace std;

namespace cartesian_abstractions {
static vector<vector<FactPair>> get_preconditions_by_operator(
    const OperatorsProxy &ops) {
    vector<vector<FactPair>> preconditions_by_operator;
    preconditions_by_operator.reserve(ops.size());
    for (OperatorProxy op : ops) {
        vector<FactPair> preconditions = task_properties::get_fact_pairs(op.get_preconditions());
        sort(preconditions.begin(), preconditions.end());
        preconditions_by_operator.push_back(move(preconditions));
    }
    return preconditions_by_operator;
}

static vector<FactPair> get_postconditions(
    const OperatorProxy &op,
    map<int, vector<CondEffect>> &cond_effects_by_op_id) {
    // Use map to obtain sorted postconditions.
    map<int, int> var_to_post;
    for (FactProxy fact : op.get_preconditions()) {
        var_to_post[fact.get_variable().get_id()] = fact.get_value();
    }
    for (EffectProxy effect : op.get_effects()) {
        FactPair fact = effect.get_fact().get_pair();
        if (effect.get_conditions().empty()) {
            // If some effect in the var has conditions, effects in the same
            // var without conditions cannot exist.
            assert(var_to_post.count(fact.var) == 0 || var_to_post[fact.var] != OP_WITH_CONDS);
            var_to_post[fact.var] = fact.value;
        } else {
            var_to_post[fact.var] = OP_WITH_CONDS;
            int op_id = op.get_id();
            vector<FactPair> conds;
            for (auto cond : effect.get_conditions()) {
                conds.push_back(cond.get_pair());
            }
            CondEffect cond_effect{conds, fact};
            if (cond_effects_by_op_id.count(op_id)) {
                cond_effects_by_op_id[op_id].push_back(cond_effect);
            } else {
                cond_effects_by_op_id[op_id] = {cond_effect};
            }
        }
    }
    vector<FactPair> postconditions;
    postconditions.reserve(var_to_post.size());
    for (const pair<const int, int> &fact : var_to_post) {
        postconditions.emplace_back(fact.first, fact.second);
    }
    return postconditions;
}

static vector<vector<FactPair>> get_postconditions_by_operator(
    const OperatorsProxy &ops,
    map<int, vector<CondEffect>> &cond_effects_by_op_id) {
    vector<vector<FactPair>> postconditions_by_operator;
    postconditions_by_operator.reserve(ops.size());
    for (OperatorProxy op : ops) {
        postconditions_by_operator.push_back(get_postconditions(op, cond_effects_by_op_id));
    }
    return postconditions_by_operator;
}

static int lookup_value(const vector<FactPair> &facts, int var) {
    assert(is_sorted(facts.begin(), facts.end()));
    for (const FactPair &fact : facts) {
        if (fact.var == var) {
            return fact.value;
        } else if (fact.var > var) {
            return UNDEFINED;
        }
    }
    return UNDEFINED;
}

static void remove_transitions_with_given_target(
    Transitions &transitions, int state_id) {
    auto new_end = remove_if(
        transitions.begin(), transitions.end(),
        [state_id](const Transition &t) {return t.target_id == state_id;});
    assert(new_end != transitions.end());
    transitions.erase(new_end, transitions.end());
}


TransitionSystem::TransitionSystem(const OperatorsProxy &ops)
    : preconditions_by_operator(get_preconditions_by_operator(ops)),
      postconditions_by_operator(get_postconditions_by_operator(ops, cond_effects_by_op_id)),
      num_non_loops(0),
      num_loops(0) {
    // This number should be big enough to store all conditional effects possible
    // values for the operator with the highest number of conditional effects
    // in the same variable.
    post_values.reserve(50);
    add_loops_in_trivial_abstraction();
}

int TransitionSystem::get_precondition_value(int op_id, int var) const {
    return lookup_value(preconditions_by_operator[op_id], var);
}

int TransitionSystem::get_postcondition_value(int op_id, int var) const {
    return lookup_value(postconditions_by_operator[op_id], var);
}

void TransitionSystem::enlarge_vectors_by_one() {
    int new_num_states = get_num_states() + 1;
    outgoing.resize(new_num_states);
    incoming.resize(new_num_states);
    loops.resize(new_num_states);
}

void TransitionSystem::add_loops_in_trivial_abstraction() {
    assert(get_num_states() == 0);
    enlarge_vectors_by_one();
    int init_id = 0;
    for (int i = 0; i < get_num_operators(); ++i) {
        add_loop(init_id, i);
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

void TransitionSystem::compute_post_values_in_children_for_cond_effects_op(const AbstractState &v1,
                                                                           const AbstractState &v2,
                                                                           int var,
                                                                           int op_id,
                                                                           int pre) {
    post_values.clear();
    // If the effect has conditions, all effects in this var must be
    // checked.
    // TODO: A more efficient implementation would probably use
    // Cartesian sets for conditions to apply intersections with the
    // source abstract state.

    // Probably too much duplicated code, but it seems the most
    // efficient way of not repeating checks for each child.
    vector<CondEffect> cond_effects = cond_effects_by_op_id[op_id];
    // The most probable thing is that some state not satisfying
    // all conditions exist in the source abstract state, and then
    // post = pre.
    // But also some states satisfying conditions can exist.
    bool some_effect_without_states_not_satisfying_conds_v1 = false;
    bool some_effect_without_states_not_satisfying_conds_v2 = false;
    for (CondEffect cond_effect : cond_effects) {
        FactPair fact = cond_effect.effect;
        if (fact.var == var) {
            bool states_satisfying_conds_v1 = false;
            bool states_satisfying_conds_v2 = false;
            bool states_not_satisfying_conds_v1 = false;
            bool states_not_satisfying_conds_v2 = false;
            bool v1_has_break = false;
            bool v2_has_break = false;
            for (FactPair cond_fact : cond_effect.conds) {
                if (cond_fact.var == var) {
                    if (!v1_has_break) {
                        if (v1.contains(cond_fact.var, cond_fact.value)) {
                            states_satisfying_conds_v1 = true;
                            if (v1.count(cond_fact.var) > 1) {
                                states_not_satisfying_conds_v1 = true;
                            }
                        } else {
                            states_not_satisfying_conds_v1 = true;
                            states_satisfying_conds_v1 = false;
                            v1_has_break = true;
                        }
                    }
                    if (!v2_has_break) {
                        if (v2.contains(cond_fact.var, cond_fact.value)) {
                            states_satisfying_conds_v2 = true;
                            if (v2.count(cond_fact.var) > 1) {
                                states_not_satisfying_conds_v2 = true;
                            }
                        } else {
                            states_not_satisfying_conds_v2 = true;
                            states_satisfying_conds_v2 = false;
                            v2_has_break = true;
                        }
                    }
                    if (v1_has_break && v2_has_break) {
                        break;
                    }
                } else {
                    // If the condition is not in the split var, any
                    // child can be used for the two children because they have
                    // the same value in that var.
                    if (v1.contains(cond_fact.var, cond_fact.value)) {
                        if (!v1_has_break) {
                            states_satisfying_conds_v1 = true;
                        }
                        if (!v2_has_break) {
                            states_satisfying_conds_v2 = true;
                        }
                        if (v1.count(cond_fact.var) > 1) {
                            if (!v1_has_break) {
                                states_not_satisfying_conds_v1 = true;
                            }
                            if (!v2_has_break) {
                                states_not_satisfying_conds_v2 = true;
                            }
                        }
                    } else {
                        if (!v1_has_break) {
                            states_not_satisfying_conds_v1 = true;
                            states_satisfying_conds_v1 = false;
                        }
                        if (!v2_has_break) {
                            states_not_satisfying_conds_v2 = true;
                            states_satisfying_conds_v2 = false;
                        }
                        break;
                    }
                }
            }
            if (!states_not_satisfying_conds_v1) {
                some_effect_without_states_not_satisfying_conds_v1 = true;
            }
            if (!states_not_satisfying_conds_v2) {
                some_effect_without_states_not_satisfying_conds_v2 = true;
            }
            if (states_satisfying_conds_v1 && states_satisfying_conds_v2) {
                post_values.push_back(CondEffectsOpPostValue {true, true, fact.value});
            } else if (states_satisfying_conds_v1) {
                post_values.push_back(CondEffectsOpPostValue {true, false, fact.value});
            } else if (states_satisfying_conds_v2) {
                post_values.push_back(CondEffectsOpPostValue {false, true, fact.value});
            }
        }
    }
    if (!some_effect_without_states_not_satisfying_conds_v1 && !some_effect_without_states_not_satisfying_conds_v2) {
        // post = pre (or undefined if no pre in var).
        post_values.push_back(CondEffectsOpPostValue {true, true, pre});
    } else if (!some_effect_without_states_not_satisfying_conds_v1) {
        // post = pre (or undefined if no pre in var).
        post_values.push_back(CondEffectsOpPostValue {true, false, pre});
    } else if (!some_effect_without_states_not_satisfying_conds_v2) {
        // post = pre (or undefined if no pre in var).
        post_values.push_back(CondEffectsOpPostValue {false, true, pre});
    }
}

void TransitionSystem::add_incoming_transitions_for_post(const AbstractState &u,
                                                         const AbstractState &v1,
                                                         const AbstractState &v2,
                                                         int var,
                                                         int op_id,
                                                         int post) {
    if (post == UNDEFINED) {
        int u_id = u.get_id();
        int v1_id = v1.get_id();
        int v2_id = v2.get_id();
        // op has no precondition and no effect on var.
        bool u_and_v1_intersect = u.domain_subsets_intersect(v1, var);
        if (u_and_v1_intersect) {
            add_transition(u_id, op_id, v1_id);
        }
        /* If u and v1 don't intersect, we must add the other transition
           and can avoid an intersection test. */
        if (!u_and_v1_intersect || u.domain_subsets_intersect(v2, var)) {
            add_transition(u_id, op_id, v2_id);
        }
    } else if (v1.contains(var, post)) {
        // op can only end in v1.
        add_transition(u.get_id(), op_id, v1.get_id());
    } else {
        // op can only end in v2.
        assert(v2.contains(var, post));
        add_transition(u.get_id(), op_id, v2.get_id());
    }
}
void TransitionSystem::add_outgoing_transitions_for_post(const AbstractState &w,
                                                         const AbstractState &v1,
                                                         const AbstractState &v2,
                                                         int var,
                                                         int op_id,
                                                         int pre,
                                                         int post,
                                                         bool for_v1,
                                                         bool for_v2) {
    assert(for_v1 || for_v2);
    if (post == UNDEFINED) {
        assert(pre == UNDEFINED);
        int v1_id = v1.get_id();
        int v2_id = v2.get_id();
        int w_id = w.get_id();
        // op has no precondition and no effect on var.
        bool v1_and_w_intersect = v1.domain_subsets_intersect(w, var);
        if (for_v1 && v1_and_w_intersect) {
            add_transition(v1_id, op_id, w_id);
        }
        /* If v1 and w don't intersect, we must add the other transition
           and can avoid an intersection test. */
        if (for_v2 && (!v1_and_w_intersect || v2.domain_subsets_intersect(w, var))) {
            add_transition(v2_id, op_id, w_id);
        }
    } else if (pre == UNDEFINED) {
        // op has no precondition, but an effect on var.
        int w_id = w.get_id();
        if (for_v1) {
            add_transition(v1.get_id(), op_id, w_id);
        }
        if (for_v2) {
            add_transition(v2.get_id(), op_id, w_id);
        }
    } else if (for_v1 && v1.contains(var, pre)) {
        // op can only start in v1.
        add_transition(v1.get_id(), op_id, w.get_id());
    } else if (for_v2 && (for_v1 || v2.contains(var, pre))) {
        // op can only start in v2 if checked for v1 (v1 does not contain pre).
        assert(v2.contains(var, pre));
        add_transition(v2.get_id(), op_id, w.get_id());
    }
}

void TransitionSystem::add_loop_for_post(const AbstractState &v1,
                                         const AbstractState &v2,
                                         int var,
                                         int op_id,
                                         int pre,
                                         int post,
                                         bool for_v1,
                                         bool for_v2) {
    assert(for_v1 || for_v2);
    int v1_id = v1.get_id();
    int v2_id = v2.get_id();
    if (pre == UNDEFINED) {
        // op has no precondition on var --> it must start in v1 and v2.
        if (post == UNDEFINED) {
            // op has no effect on var --> it must end in v1 and v2.
            if (for_v1) {
                add_loop(v1_id, op_id);
            }
            if (for_v2) {
                add_loop(v2_id, op_id);
            }
        } else if (for_v2 && v2.contains(var, post)) {
            // op must end in v2.
            add_transition(v1_id, op_id, v2_id);
            add_loop(v2_id, op_id);
        } else if (for_v1 && (for_v2 || v1.contains(var, post))) {
            // op must end in v1.
            assert(v1.contains(var, post));
            add_loop(v1_id, op_id);
            add_transition(v2_id, op_id, v1_id);
        }
    } else if (v1.contains(var, pre)) {
        // op must start in v1.
        assert(post != UNDEFINED);
        if (for_v1 && v1.contains(var, post)) {
            // op must end in v1.
            add_loop(v1_id, op_id);
        } else if (for_v2 && (for_v1 || v2.contains(var, post))) {
            // op must end in v2.
            assert(v2.contains(var, post));
            add_transition(v1_id, op_id, v2_id);
        }
    } else {
        // op must start in v2.
        assert(v2.contains(var, pre));
        assert(post != UNDEFINED);
        if (for_v1 && v1.contains(var, post)) {
            // op must end in v1.
            add_transition(v2_id, op_id, v1_id);
        } else if (for_v2 && (for_v1 || v2.contains(var, post))) {
            // op must end in v2.
            assert(v2.contains(var, post));
            add_loop(v2_id, op_id);
        }
    }
}

void TransitionSystem::rewire_incoming_transitions(
    const Transitions &old_incoming, const AbstractStates &states, int v_id,
    const AbstractState &v1, const AbstractState &v2, int var) {
    /* State v has been split into v1 and v2. Now for all transitions
       u->v we need to add transitions u->v1, u->v2, or both. */
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
        int post = get_postcondition_value(op_id, var);
        if (post == OP_WITH_CONDS) {
            // If the effect has conditions, all effects in this var must be
            // checked.
            // TODO: A more efficient implementation would probably use
            // Cartesian sets for conditions to apply intersections with the
            // source abstract state.

            vector<CondEffect> cond_effects = cond_effects_by_op_id[op_id];
            // The most probable thing is that some state not satisfying
            // all conditions exist in the source abstract state, and then
            // post = pre. If for some effect all states satisfy all
            // conditions, then the post=pre case does not happen.
            // But also some states satisfying conditions can exist.
            bool some_effect_without_states_not_satisfying_conds = false;
            bool states_not_satisfying_conds = false;
            for (CondEffect cond_effect : cond_effects) {
                FactPair fact = cond_effect.effect;
                if (fact.var == var) {
                    bool states_satisfying_conds = true;
                    for (FactPair cond_fact : cond_effect.conds) {
                        if (u.contains(cond_fact.var, cond_fact.value)) {
                            if (u.count(cond_fact.var) > 1) {
                                states_not_satisfying_conds = true;
                            }
                        } else {
                            states_not_satisfying_conds = true;
                            states_satisfying_conds = false;
                            break;
                        }
                    }
                    if (!states_not_satisfying_conds) {
                        some_effect_without_states_not_satisfying_conds = true;
                    }
                    if (states_satisfying_conds) {
                        post = fact.value;
                        add_incoming_transitions_for_post(u, v1, v2, var, op_id, post);
                    }
                }
            }
            if (!some_effect_without_states_not_satisfying_conds) {
                // post = pre (or undefined if no pre in var).
                post = get_precondition_value(op_id, var);
                add_incoming_transitions_for_post(u, v1, v2, var, op_id, post);
            }
        } else {
            add_incoming_transitions_for_post(u, v1, v2, var, op_id, post);
        }
    }
}

void TransitionSystem::rewire_outgoing_transitions(
    const Transitions &old_outgoing, const AbstractStates &states, int v_id,
    const AbstractState &v1, const AbstractState &v2, int var) {
    /* State v has been split into v1 and v2. Now for all transitions
       v->w we need to add transitions v1->w, v2->w, or both. */
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
        int pre = get_precondition_value(op_id, var);
        int post = get_postcondition_value(op_id, var);
        if (post == OP_WITH_CONDS) {
            compute_post_values_in_children_for_cond_effects_op(v1, v2, var, op_id, pre);
            for (const CondEffectsOpPostValue &post : post_values) {
                add_outgoing_transitions_for_post(w, v1, v2, var, op_id, pre, post.value, post.for_v1, post.for_v2);
            }
        } else {
            add_outgoing_transitions_for_post(w, v1, v2, var, op_id, pre, post);
        }
    }
}

void TransitionSystem::rewire_loops(
    const Loops &old_loops, const AbstractState &v1, const AbstractState &v2, int var) {
    /* State v has been split into v1 and v2. Now for all self-loops
       v->v we need to add one or two of the transitions v1->v1, v1->v2,
       v2->v1 and v2->v2. */
    for (int op_id : old_loops) {
        int pre = get_precondition_value(op_id, var);
        int post = get_postcondition_value(op_id, var);
        if (post == OP_WITH_CONDS) {
            compute_post_values_in_children_for_cond_effects_op(v1, v2, var, op_id, pre);
            for (const CondEffectsOpPostValue &post : post_values) {
                add_loop_for_post(v1, v2, var, op_id, pre, post.value, post.for_v1, post.for_v2);
            }
        } else {
            add_loop_for_post(v1, v2, var, op_id, pre, post);
        }
    }
    num_loops -= old_loops.size();
}

void TransitionSystem::rewire(
    const AbstractStates &states, int v_id,
    const AbstractState &v1, const AbstractState &v2, int var) {
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

    // Remove old transitions and add new transitions.
    rewire_incoming_transitions(old_incoming, states, v_id, v1, v2, var);
    rewire_outgoing_transitions(old_outgoing, states, v_id, v1, v2, var);
    rewire_loops(old_loops, v1, v2, var);
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

const vector<FactPair> &TransitionSystem::get_preconditions(int op_id) const {
    assert(utils::in_bounds(op_id, preconditions_by_operator));
    return preconditions_by_operator[op_id];
}

int TransitionSystem::get_num_states() const {
    assert(incoming.size() == outgoing.size());
    assert(loops.size() == outgoing.size());
    return outgoing.size();
}

int TransitionSystem::get_num_operators() const {
    return preconditions_by_operator.size();
}

int TransitionSystem::get_num_non_loops() const {
    return num_non_loops;
}

int TransitionSystem::get_num_loops() const {
    return num_loops;
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
