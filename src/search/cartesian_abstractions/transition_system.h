#ifndef CARTESIAN_ABSTRACTIONS_TRANSITION_SYSTEM_H
#define CARTESIAN_ABSTRACTIONS_TRANSITION_SYSTEM_H

#include "abstract_state.h"
#include "types.h"

#include "../task_proxy.h"

#include <map>
#include <vector>

struct FactPair;
class OperatorsProxy;

namespace utils {
class LogProxy;
}

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
enum AddTransitionToChild {
    FIRST,
    SECOND,
    FROM_FIRST_TO_SECOND,
    FROM_SECOND_TO_FIRST,
    FROM_FIRST_TO_SECOND_AND_SECOND,
    FROM_SECOND_TO_FIRST_AND_FIRST,
    BOTH,
    NONE,
};
/*
  Rewire transitions after each split.
*/
class TransitionSystem {
    std::unordered_map<int, std::vector<CondEffect>> cond_effects_by_op_id = {};
    const std::vector<std::vector<FactPair>> preconditions_by_operator;
    const std::vector<std::vector<FactPair>> postconditions_by_operator;
    // Vector used to store post values for each child for ops with conditional
    // effects.
    std::vector<CondEffectsOpPostValue> post_values;

    // Transitions from and to other abstract states.
    std::vector<Transitions> incoming;
    std::vector<Transitions> outgoing;

    // Store self-loops (operator indices) separately to save space.
    std::vector<Loops> loops;

    int num_non_loops;
    int num_loops;

    void enlarge_vectors_by_one();

    // Add self-loops to single abstract state in trivial abstraction.
    void add_loops_in_trivial_abstraction();

    int get_precondition_value(int op_id, int var) const;
    int get_postcondition_value(int op_id, int var) const;
    void compute_post_values_in_children_for_cond_effects_op(const AbstractState &v1,
                                                             const AbstractState &v2,
                                                             int var,
                                                             int op_id,
                                                             int pre);

    void add_transition(int src_id, int op_id, int target_id);
    void add_loop(int state_id, int op_id);

    AddTransitionToChild get_incoming_transitions_for_post(const AbstractState &u,
                                                           const AbstractState &v1,
                                                           const AbstractState &v2,
                                                           int var,
                                                           int post);
    AddTransitionToChild get_outgoing_transitions_for_post(const AbstractState &w,
                                                           const AbstractState &v1,
                                                           const AbstractState &v2,
                                                           int var,
                                                           int pre,
                                                           int post,
                                                           bool for_v1 = true,
                                                           bool for_v2 = true);
    AddTransitionToChild get_loops_and_intertransitions_for_post(const AbstractState &v1,
                                                                 const AbstractState &v2,
                                                                 int var,
                                                                 int pre,
                                                                 int post,
                                                                 bool for_v1 = true,
                                                                 bool for_v2 = true);

    void rewire_incoming_transitions(
        const Transitions &old_incoming, const AbstractStates &states,
        int v_id, const AbstractState &v1, const AbstractState &v2, int var);
    void rewire_outgoing_transitions(
        const Transitions &old_outgoing, const AbstractStates &states,
        int v_id, const AbstractState &v1, const AbstractState &v2, int var);
    void rewire_loops(
        const Loops &old_loops,
        const AbstractState &v1, const AbstractState &v2, int var);

public:
    explicit TransitionSystem(const OperatorsProxy &ops);

    // Update transition system after v has been split for var into v1 and v2.
    void rewire(
        const AbstractStates &states, int v_id,
        const AbstractState &v1, const AbstractState &v2, int var);

    const std::vector<Transitions> &get_incoming_transitions() const;
    const std::vector<Transitions> &get_outgoing_transitions() const;
    const std::vector<Loops> &get_loops() const;

    const std::vector<FactPair> &get_preconditions(int op_id) const;

    int get_num_states() const;
    int get_num_operators() const;
    int get_num_non_loops() const;
    int get_num_loops() const;

    void print_statistics(utils::LogProxy &log) const;
    void dump() const;
};
}

#endif
