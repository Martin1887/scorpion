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

struct TransitionElements {
    int src_id;
    int op_id;
    int target_id;

    bool operator==(const TransitionElements &other) const {
        return src_id == other.src_id && op_id == other.op_id && target_id == other.target_id;
    }
};

struct NonSpuriousTransitionsCache {
    utils::HashSet<TransitionElements> non_spurious_transitions_cache{};
    utils::HashMap<int, std::vector<TransitionElements>> cached_transitions_per_state{};

    void add(const TransitionElements &tr) {
        non_spurious_transitions_cache.insert(tr);
        if (cached_transitions_per_state.contains(tr.src_id)) {
            std::vector<TransitionElements> &cached = cached_transitions_per_state.at(tr.src_id);
            cached.push_back(tr);
            cached_transitions_per_state.insert_or_assign(tr.src_id, move(cached));
        } else {
            std::vector<TransitionElements> cached = {tr};
            cached_transitions_per_state.insert_or_assign(tr.src_id, move(cached));
        }
        if (cached_transitions_per_state.contains(tr.target_id)) {
            std::vector<TransitionElements> &cached = cached_transitions_per_state.at(tr.target_id);
            cached.push_back(tr);
            cached_transitions_per_state.insert_or_assign(tr.target_id, move(cached));
        } else {
            std::vector<TransitionElements> cached = {tr};
            cached_transitions_per_state.insert_or_assign(tr.target_id, move(cached));
        }
    }

    void remove(int abstract_state_id) {
        if (cached_transitions_per_state.contains(abstract_state_id)) {
            for (const TransitionElements &tr : cached_transitions_per_state.at(abstract_state_id)) {
                non_spurious_transitions_cache.erase(tr);
            }
            cached_transitions_per_state.erase(abstract_state_id);
        }
    }

    bool contains(const TransitionElements &tr) {
        return non_spurious_transitions_cache.contains(tr);
    }
};

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
    bool remove_plan_spurious_transitions;
    const bool refine_init;

    std::shared_ptr<MutexInformation> mutex_information;
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

    int removed_optimal_plan_transitions = 0;

    NonSpuriousTransitionsCache non_spurious_transitions_cache;

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

    bool remove_first_invalid_transition(std::unique_ptr<Solution> &solution,
                                         utils::Timer &update_distances_timer);
    std::unique_ptr<Solution> get_optimal_abstract_solution(utils::Timer &update_distances_timer);

    void update_shortest_paths_incrementally(const std::vector<Transitions> &in,
                                             const std::vector<Transitions> &out,
                                             int v, int v1, int v2, bool disambiguated,
                                             Transitions old_incoming, Transitions old_outgoing,
                                             const std::unordered_set<int> &goals,
                                             const int initial_state,
                                             utils::Timer &update_distances_timer);

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
        bool remove_plan_spurious_transitions,
        bool refine_init,
        lp::LPSolverType lp_solver,
        std::shared_ptr<disambiguation::DisambiguationMethod> &abstract_space_disambiguation,
        std::shared_ptr<disambiguation::DisambiguationMethod> &flaw_search_states_disambiguation,
        std::shared_ptr<std::vector<disambiguation::DisambiguatedOperator>> _operators,
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

namespace utils {
inline void feed(HashState &hash_state, const cartesian_abstractions::TransitionElements &val) {
    feed(hash_state, val.src_id);
    feed(hash_state, val.op_id);
    feed(hash_state, val.target_id);
}
}

#endif
