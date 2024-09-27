#ifndef CARTESIAN_ABSTRACTIONS_CEGAR_H
#define CARTESIAN_ABSTRACTIONS_CEGAR_H

#include "flaw_search.h"
#include "refinement_hierarchy.h"
#include "split_selector.h"
#include "types.h"

#include "../task_proxy.h"

#include "../task_utils/mutex_information.h"
#include "../utils/countdown_timer.h"

#include <memory>

namespace utils {
class RandomNumberGenerator;
class LogProxy;
}

namespace disambiguation {
class DisambiguationMethod;
}

namespace cartesian_abstractions {
class Abstraction;
enum class DotGraphVerbosity;
class ShortestPaths;

/*
  Iteratively refine a Cartesian abstraction with counterexample-guided
  abstraction refinement (CEGAR).

  Store the abstraction, use AbstractSearch to find abstract solutions, find
  flaws, use SplitSelector to select splits in case of ambiguities and break
  spurious solutions.
*/
class CEGAR {
    const TaskProxy task_proxy;
    const std::vector<int> domain_sizes;
    const int max_states;
    const int max_non_looping_transitions;
    const PickFlawedAbstractState pick_flawed_abstract_state;

    std::shared_ptr<MutexInformation> mutex_information;
    std::shared_ptr<disambiguation::DisambiguationMethod> operators_disambiguation;
    std::shared_ptr<disambiguation::DisambiguationMethod> abstract_space_disambiguation;
    std::shared_ptr<disambiguation::DisambiguationMethod> flaw_search_states_disambiguation;
    std::shared_ptr<std::vector<disambiguation::DisambiguatedOperator>> operators;

    std::unique_ptr<Abstraction> abstraction;
    // Transition system used for simulations, from which only transitions are
    // updated improving massively the performance.
    std::shared_ptr<TransitionSystem> simulated_transition_system;
    std::unique_ptr<ShortestPaths> shortest_paths;
    std::unique_ptr<FlawSearch> flaw_search;

    // Limit the time for building the abstraction.
    utils::CountdownTimer timer;
    double max_time;

    utils::LogProxy &log;
    const DotGraphVerbosity dot_graph_verbosity;

    // Only used for logging progress.
    int old_abstract_solution_cost = -1;

    bool may_keep_refining(bool in_current_direction = false) const;

    /*
      Map all states that can only be reached after reaching the goal
      fact to arbitrary goal states.

      We need this method only for landmark subtasks, but calling it
      for other subtasks with a single goal fact doesn't hurt and
      simplifies the implementation.
    */
    void separate_facts_unreachable_before_goal(bool refine_goals) const;

    // Build abstraction.
    void refinement_loop();

    void print_statistics() const;

public:
    CEGAR(
        const std::shared_ptr<AbstractTask> &task,
        int max_states,
        int max_non_looping_transitions,
        double max_time,
        PickFlawedAbstractState pick_flawed_abstract_state,
        PickSplit pick_split,
        FilterSplit filter_split,
        PickSplit tiebreak_split,
        PickSequenceFlaw sequence_split,
        PickSequenceFlaw sequence_tiebreak_split,
        int max_concrete_states_per_abstract_state,
        int max_state_expansions,
        bool intersect_flaw_search_abstract_states,
        lp::LPSolverType lp_solver,
        std::shared_ptr<disambiguation::DisambiguationMethod> &operators_disambiguation,
        std::shared_ptr<disambiguation::DisambiguationMethod> &abstract_space_disambiguation,
        std::shared_ptr<disambiguation::DisambiguationMethod> &flaw_search_states_disambiguation,
        utils::RandomNumberGenerator &rng,
        utils::LogProxy &log,
        DotGraphVerbosity dot_graph_verbosity);
    ~CEGAR();

    CEGAR(const CEGAR &) = delete;

    std::unique_ptr<Abstraction> extract_abstraction();

    void print_useless_refinements(const RefinementHierarchy &hier, const std::vector<int> &goal_distances) const;
};

Cost get_optimal_plan_cost(const Solution &solution, TaskProxy task_proxy);
}

#endif
