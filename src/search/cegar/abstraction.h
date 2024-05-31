#ifndef CEGAR_ABSTRACTION_H
#define CEGAR_ABSTRACTION_H

#include "cartesian_set.h"
#include "transition_system.h"
#include "types.h"

#include "../task_proxy.h"

#include "../utils/collections.h"

#include <memory>
#include <vector>

namespace utils {
class LogProxy;
}

namespace cegar {
class AbstractState;
class RefinementHierarchy;
class TransitionSystem;

struct AbstractStateSplit {
    int v1_id;
    int v2_id;
    std::vector<int> v2_values;
    CartesianSet v1_cartesian_set;
    CartesianSet v2_cartesian_set;
};

class SimulatedRefinement {
public:
    std::shared_ptr<TransitionSystem> transition_system;
    Goals goals;
    const int v1_id;
    const int v2_id;
    SimulatedRefinement(const std::shared_ptr<TransitionSystem> tr,
                        const Goals goals,
                        const int v1_id,
                        const int v2_id)
        : transition_system(tr),
          goals(goals),
          v1_id(v1_id),
          v2_id(v2_id) {
    }
};

/*
  Store the set of AbstractStates, use AbstractSearch to find abstract
  solutions, find flaws, use SplitSelector to select splits in case of
  ambiguities, break spurious solutions and maintain the
  RefinementHierarchy.
*/
class Abstraction {
    const TaskProxy task_proxy;
    const std::unique_ptr<TransitionSystem> transition_system;
    const State concrete_initial_state;
    const std::vector<FactPair> goal_facts;

    // All (as of yet unsplit) abstract states.
    AbstractStates states;
    // State ID of abstract initial state.
    int init_id;
    // Abstract goal states. Only landmark tasks can have multiple goal states.
    Goals goals;

    /* DAG with inner nodes for all split states and leaves for all
       current states. */
    std::unique_ptr<RefinementHierarchy> refinement_hierarchy;

    utils::LogProxy &log;

    void initialize_trivial_abstraction(const std::vector<int> &domain_sizes);

    AbstractStateSplit split(
        const AbstractState &state, int var, const std::vector<int> &wanted) const;

public:
    Abstraction(const std::shared_ptr<AbstractTask> &task, utils::LogProxy &log);
    ~Abstraction();

    Abstraction(const Abstraction &) = delete;

    int get_num_states() const;
    const AbstractState &get_initial_state() const;
    const Goals &get_goals() const;
    const AbstractState &get_state(int state_id) const;
    int get_abstract_state_id(const State &state) const;
    const TransitionSystem &get_transition_system() const;
    std::unique_ptr<RefinementHierarchy> extract_refinement_hierarchy();

    /* Needed for CEGAR::separate_facts_unreachable_before_goal(). */
    void mark_all_states_as_goals();

    // Split state into two child states.
    std::pair<int, int> refine(
        const AbstractState &state, int var, const std::vector<int> &wanted);
    SimulatedRefinement simulate_refinement(
        std::shared_ptr<TransitionSystem> &simulated_transition_system,
        const AbstractState &state,
        int var,
        const std::vector<int> &wanted) const;

    void print_statistics() const;

    void dump() const;

    // Print the distribution of h values as a vector with the numbers to
    // multiply to get the final number of states
    void h_distribution(const std::vector<int> &goal_distances,
                        const std::vector<int> &init_distances) const;
};
}

#endif
