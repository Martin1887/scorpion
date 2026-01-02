#ifndef CARTESIAN_ABSTRACTIONS_TRANSITION_REWIRER_H
#define CARTESIAN_ABSTRACTIONS_TRANSITION_REWIRER_H

#include "types.h"

#include "cartesian_set.h"
#include "../utils/collections.h"

#include <cassert>
#include <deque>
#include <vector>

struct FactPair;
class OperatorsProxy;

namespace cartesian_abstractions {
struct CondEffectsOpPostValue {
    bool for_v1;
    bool for_v2;
    int value;
};
struct CondEffect {
    std::vector<FactPair> conds;
    FactPair effect;
};
struct AddTransitionTo {
    bool first = false;
    bool second = false;
    bool first_loop = false;
    bool second_loop = false;

    void reset() {
        first = false;
        second = false;
        first_loop = false;
        second_loop = false;
    }
};
class TransitionRewirer {
    const int n_vars;
    AddTransitionTo add_transition_to;
    const std::vector<std::vector<CondEffect>> cond_effects_by_op;
    const std::vector<std::vector<FactPair>> uncond_effects_by_op;
    const std::vector<std::vector<bool>> exists_effect_condition_in_var_by_op;
    const std::vector<std::vector<bool>> exists_effect_in_var_by_op;
    const std::vector<std::vector<FactPair>> preconditions_by_operator;
    const std::vector<std::vector<FactPair>> postconditions_by_operator;
    const std::vector<std::vector<std::unordered_set<int>>> postcondition_set_by_operator;
    // Variables used to not allocate new objects to each call to
    // compute_partial_post_cartesian_set.
    CartesianSet partial_post_set;
    std::vector<bool> affected_vars;
    std::vector<bool> vars_changed;

    bool exists_outgoing_transition(const AbstractState &target);
    void update_incoming_transitions_for_post(const AbstractState &u,
                                              const AbstractState &v1,
                                              const AbstractState &v2,
                                              int var,
                                              int post,
                                              bool with_condition = false);
    void update_outgoing_transitions_for_post(const AbstractState &w,
                                              const AbstractState &v1,
                                              const AbstractState &v2,
                                              int var,
                                              int pre,
                                              int post,
                                              bool with_condition = false);
    void update_loops_and_intertransitions_for_post(const AbstractState &v1,
                                                    const AbstractState &v2,
                                                    int var,
                                                    int pre,
                                                    int post,
                                                    bool with_condition = false);
    void compute_partial_post_cartesian_set(const AbstractState &child,
                                            const AbstractState &target,
                                            int op_id,
                                            int var);

    Transitions rewire_incoming_transitions(
        std::deque<Transitions> &incoming, std::deque<Transitions> &outgoing,
        const AbstractStates &states, int v_id,
        const AbstractState &v1, const AbstractState &v2, int var);
    Transitions rewire_outgoing_transitions(
        std::deque<Transitions> &incoming, std::deque<Transitions> &outgoing,
        const AbstractStates &states, int v_id,
        const AbstractState &v1, const AbstractState &v2, int var);

public:
    explicit TransitionRewirer(const OperatorsProxy &ops,
                               const std::vector<int> &domain_sizes);

    std::tuple<Transitions, Transitions> rewire_transitions(
        std::deque<Transitions> &incoming, std::deque<Transitions> &outgoing,
        const AbstractStates &states, int v_id,
        const AbstractState &v1, const AbstractState &v2, int var);

    void rewire_loops(
        std::deque<Loops> &loops,
        std::deque<Transitions> &incoming, std::deque<Transitions> &outgoing,
        int v_id, const AbstractState &v1, const AbstractState &v2, int var);

    const std::vector<FactPair> &get_preconditions(int op_id) const {
        assert(utils::in_bounds(op_id, preconditions_by_operator));
        return preconditions_by_operator[op_id];
    }
    const std::vector<FactPair> &get_postconditions(int op_id) const {
        assert(utils::in_bounds(op_id, postconditions_by_operator));
        return postconditions_by_operator[op_id];
    }
    const std::vector<std::unordered_set<int>> &get_postcondition_set(int op_id) const {
        assert(utils::in_bounds(op_id, postcondition_set_by_operator));
        return postcondition_set_by_operator[op_id];
    }
    const std::vector<Facts> &get_preconditions() const {
        return preconditions_by_operator;
    }
    const std::vector<Facts> &get_postconditions() const {
        return postconditions_by_operator;
    }
    const std::vector<std::vector<CondEffect>> &get_cond_effects_by_op() const {
        return cond_effects_by_op;
    }
    const std::vector<std::vector<FactPair>> &get_uncond_effects_by_op() const {
        return uncond_effects_by_op;
    }
    const std::vector<bool> &exists_effect_in_var(int op_id) const {
        return exists_effect_in_var_by_op[op_id];
    }
    int get_precondition_value(int op_id, int var) const;
    int get_postcondition_value(int op_id, int var) const;

    int get_num_operators() const;
};
}

#endif
