#ifndef CARTESIAN_ABSTRACTIONS_TRANSITION_SYSTEM_H
#define CARTESIAN_ABSTRACTIONS_TRANSITION_SYSTEM_H

#include "types.h"

#include "../task_proxy.h"

#include <vector>

struct FactPair;

namespace cartesian_state {
class CartesianState;
}

namespace disambiguation {
class DisambiguatedOperator;
}

namespace utils {
class LogProxy;
}

namespace cartesian_abstractions {
/*
  Rewire transitions after each split.
*/
class TransitionSystem {
    const std::shared_ptr<std::vector<disambiguation::DisambiguatedOperator>> operators;

    // Transitions from and to other abstract states.
    std::vector<Transitions> incoming;
    std::vector<Transitions> outgoing;

    // Store self-loops (operator indices) separately to save space.
    std::vector<Loops> loops;

    int num_non_loops;
    int num_loops;

    void enlarge_vectors_by_one();

    void add_transition(int src_id, int op_id, int target_id);
    void add_loop(int state_id, int op_id);

    void rewire_incoming_transitions(
        const Transitions &old_incoming, const AbstractStates &states,
        int v_id, const AbstractState &v1, const AbstractState &v2);
    void rewire_outgoing_transitions(
        const Transitions &old_outgoing, const AbstractStates &states,
        int v_id, const AbstractState &v1, const AbstractState &v2);
    void rewire_loops(
        const Loops &old_loops,
        const AbstractState &v1, const AbstractState &v2,
        const bool simulated = false);

public:
    explicit TransitionSystem(const std::shared_ptr<std::vector<disambiguation::DisambiguatedOperator>> &ops);

    // Update transition system after v has been split for var into v1 and v2.
    std::tuple<Transitions, Transitions> rewire(
        const AbstractStates &states, int v_id,
        const AbstractState &v1, const AbstractState &v2,
        const bool simulated = false);
    // Add self-loops to single abstract state in trivial abstraction.
    void add_loops_in_trivial_abstraction(const AbstractState &init,
                                          const bool disambiguated);

    const std::vector<Transitions> &get_incoming_transitions() const;
    const std::vector<Transitions> &get_outgoing_transitions() const;
    const std::vector<Loops> &get_loops() const;

    // Force a set of transitions, used to update another transition system
    // used for simulations.
    void force_new_transitions(const std::vector<Transitions> &new_incoming,
                               const std::vector<Transitions> &new_outgoing,
                               const std::vector<Loops> &new_loops);

    const cartesian_state::CartesianState &get_preconditions(int op_id) const;

    int get_num_states() const;
    int get_num_operators() const;
    int get_num_non_loops() const;
    int get_num_loops() const;
    const std::shared_ptr<std::vector<disambiguation::DisambiguatedOperator>> &get_operators() const;

    void print_statistics(utils::LogProxy &log) const;
    void dump() const;
};
}

#endif
