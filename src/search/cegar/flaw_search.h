#ifndef CEGAR_FLAW_SEARCH_H
#define CEGAR_FLAW_SEARCH_H

#include "flaw.h"
#include "abstract_state.h"
#include "split_selector.h"
#include "types.h"

// Needed for SearchStatus enum.
#include "../search_engine.h"

#include "../utils/logging.h"
#include "../utils/timer.h"

#include <parallel_hashmap/phmap.h>

#include <stack>

namespace utils {
class CountdownTimer;
class LogProxy;
class RandomNumberGenerator;
}

namespace cegar {
class Abstraction;
class ShortestPaths;

// Variants from ICAPS 2022 paper (in order): FIRST, MIN_H, MAX_H, MIN_H, BATCH_MIN_H.
enum class PickFlawedAbstractState {
    // Consider first encountered flawed abstract state + a random concrete state.
    FIRST,
    // Legacy code: follow the arbitrary solution in shortest path tree (no flaw search).
    // Consider first encountered flawed abstract state + a random concrete state.
    FIRST_ON_SHORTEST_PATH,
    // Legacy code: follow the arbitrary solution in shortest path tree (no flaw search)
    // splitting the unwanted values.
    // Consider first encountered flawed abstract state + a random concrete state.
    FIRST_ON_SHORTEST_PATH_UNWANTED_VALUES,
    // Follow the arbitrary solution in shortest path in backward direction
    // (from the goal) splitting the unwanted values.
    FIRST_ON_SHORTEST_PATH_BACKWARD,
    // Follow the arbitrary solution in shortest path in backward direction
    // (from the goal) splitting the wanted values.
    FIRST_ON_SHORTEST_PATH_BACKWARD_WANTED_VALUES,
    // Follow the arbitrary solution in shortest path in backward direction
    // (from the goal) splitting the wanted values refining the init state
    // before refinement steps.
    FIRST_ON_SHORTEST_PATH_BACKWARD_WANTED_VALUES_REFINING_INIT_STATE,
    // Follow the arbitrary solution in shortest path in backward and forward
    // directions interleaving them.
    FIRST_ON_SHORTEST_PATH_BIDIRECTIONAL_INTERLEAVED,
    // Follow the arbitrary solution in shortest path in backward direction the
    // first half of the time/states/transitions and in the forward direction the other one.
    FIRST_ON_SHORTEST_PATH_BIDIRECTIONAL_BACKWARD_FORWARD,
    // Follow the arbitrary solution in shortest path in forward direction the
    // first half of the time/states/transitions and in the backward direction the other one.
    FIRST_ON_SHORTEST_PATH_BIDIRECTIONAL_FORWARD_BACKWARD,
    // Follow the arbitrary solution in shortest path in the direction where
    // the found flaw is closer to the goal.
    FIRST_ON_SHORTEST_PATH_BIDIRECTIONAL_CLOSEST_TO_GOAL,
    // Collect all flawed abstract states.
    // Consider a random abstract state + a random concrete state.
    RANDOM,
    // Collect all flawed abstract states.
    // Consider a random abstract state with min h + a random concrete state.
    MIN_H,
    // Collect all flawed abstract states.
    // Consider a random abstract state with max h + a random concrete state.
    MAX_H,
    // Collect all flawed abstract states and iteratively refine them (by increasing h value).
    // Only start a new flaw search once all remaining flawed abstract states are refined.
    // For each abstract state consider all concrete states.
    BATCH_MIN_H
};

using OptimalTransitions = phmap::flat_hash_map<int, std::vector<int>>;

struct ForwardLegacyFlaw {
    StateID concrete_state_id;
    int abstract_state_id;
    bool split_goal_state;

    ForwardLegacyFlaw(StateID concrete_state_id,
                      int abstract_state_id,
                      bool split_goal_state)
        : concrete_state_id(concrete_state_id),
          abstract_state_id(abstract_state_id),
          split_goal_state(split_goal_state){};
};
struct BackwardLegacyFlaw {
    AbstractState flaw_search_state;
    int abstract_state_id;
    bool split_init_state;

    BackwardLegacyFlaw(AbstractState flaw_search_state,
                       int abstract_id,
                       bool split_init_state)
        : flaw_search_state(flaw_search_state),
          abstract_state_id(abstract_id),
          split_init_state(split_init_state) {};
};
struct SplitAndDirection {
    std::unique_ptr<Split> split;
    bool backward_direction;

    SplitAndDirection(std::unique_ptr<Split> split, bool backward_direction)
        : split(std::move(split)), backward_direction(backward_direction) {};
};

class FlawSearch {
    TaskProxy task_proxy;
    const std::vector<int> domain_sizes;
    const Abstraction &abstraction;
    const ShortestPaths &shortest_paths;
    const SplitSelector split_selector;
    utils::RandomNumberGenerator &rng;
    const PickFlawedAbstractState pick_flawed_abstract_state;
    const int max_concrete_states_per_abstract_state;
    const int max_state_expansions;
    // Intersect flaw search states with the mapped one to find more flaws.
    const bool intersect_flaw_search_abstract_states;
    mutable utils::LogProxy log;
    mutable utils::LogProxy silent_log;  // For concrete search space.

    static const int MISSING = -1;

    // Search data
    std::stack<StateID> open_list;
    std::unique_ptr<StateRegistry> state_registry;
    std::unique_ptr<SearchSpace> search_space;
    std::unique_ptr<PerStateInformation<int>> cached_abstract_state_ids;

    // Flaw data
    FlawedState last_refined_flawed_state;
    Cost best_flaw_h;
    FlawedStates flawed_states;

    // Statistics
    int num_searches;
    int num_overall_expanded_concrete_states;
    int max_expanded_concrete_states;
    utils::Timer flaw_search_timer;
    utils::Timer compute_splits_timer;
    utils::Timer pick_split_timer;

    int get_abstract_state_id(const State &state) const;
    Cost get_h_value(int abstract_state_id) const;
    void add_flaw(int abs_id, const State &state);
    OptimalTransitions get_f_optimal_transitions(int abstract_state_id) const;
    OptimalTransitions get_f_optimal_backward_transitions(int abstract_state_id) const;

    void initialize();
    SearchStatus step();
    SearchStatus search_for_flaws(const utils::CountdownTimer &cegar_timer);

    std::unique_ptr<Split> create_split(
        const std::vector<StateID> &state_ids, int abstract_state_id, bool split_unwanted_values);
    std::unique_ptr<Split> create_split_from_goal_state(
        const std::vector<StateID> &state_ids, int abstract_state_id, bool split_unwanted_values);
    std::unique_ptr<Split> create_backward_split(
        const std::vector<AbstractState> &states, int abstract_state_id, bool split_unwanted_values);
    std::unique_ptr<Split> create_backward_split_from_init_state(
        const std::vector<AbstractState> &states, int abstract_state_id, bool split_unwanted_values);

    FlawedState get_flawed_state_with_min_h();
    std::unique_ptr<Split> get_single_split(const utils::CountdownTimer &cegar_timer);
    std::unique_ptr<Split> get_min_h_batch_split(const utils::CountdownTimer &cegar_timer);

    // Return concrete state id and abstract state id where create the split.
    std::unique_ptr<ForwardLegacyFlaw> get_split_legacy_forward(const Solution &solution);
    // Return pseudo-concrete state id and abstract state id where create the split.
    std::unique_ptr<BackwardLegacyFlaw> get_split_legacy_backward(const Solution &solution);

public:
    FlawSearch(
        const std::shared_ptr<AbstractTask> &task,
        const Abstraction &abstraction,
        const ShortestPaths &shortest_paths,
        utils::RandomNumberGenerator &rng,
        PickFlawedAbstractState pick_flawed_abstract_state,
        PickSplit pick_split,
        PickSplit tiebreak_split,
        int max_concrete_states_per_abstract_state,
        int max_state_expansions,
        bool intersect_flaw_search_abstract_states,
        const utils::LogProxy &log);

    std::unique_ptr<Split> get_split(const utils::CountdownTimer &cegar_timer);
    std::unique_ptr<Split> get_split_legacy(const Solution &solution,
                                            const bool backward = false,
                                            const bool split_unwanted_values = false);
    SplitAndDirection get_split_legacy_closest_to_goal(const Solution &solution,
                                                      const bool split_unwanted_values);

    void print_statistics() const;
};
}

#endif
