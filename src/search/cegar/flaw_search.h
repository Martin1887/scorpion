#ifndef CEGAR_FLAW_SEARCH_H
#define CEGAR_FLAW_SEARCH_H

#include "abstract_state.h"
#include "cartesian_set.h"
#include "flaw.h"
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

using utils::HashMap;

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
    BATCH_MIN_H,
    // Sequence flaws in the forward direction splitting wanted values.
    SEQUENCE,
    // Sequence flaws in the forward direction only over the abstraction
    // (without taking into account init state nor goals) splitting wanted values.
    SEQUENCE_IN_ABSTRACTION,
    // Sequence flaws in the backward direction splitting unwanted values.
    SEQUENCE_BACKWARD,
    // Sequence flaws in the backward direction only over the abstraction
    // (without taking into account init state nor goals) splitting unwanted values.
    SEQUENCE_IN_ABSTRACTION_BACKWARD,
    // Sequence flaws in both directions splitting wanted values in the forward
    // direction and unwanted values in the backward direction.
    SEQUENCE_BIDIRECTIONAL,
    // Sequence flaws in both directions only over the abstraction
    // (without taking into account init state nor goals) splitting wanted values
    // in the forward direction and unwanted values in the backward direction.
    SEQUENCE_IN_ABSTRACTION_BIDIRECTIONAL
};

using OptimalTransitions = phmap::flat_hash_map<int, std::vector<int>>;

struct LegacyFlaw {
    AbstractState flaw_search_state;
    int abstract_state_id;
    bool split_last_state;

    LegacyFlaw(AbstractState flaw_search_state,
               int abstract_id,
               bool split_last_state)
        : flaw_search_state(flaw_search_state),
          abstract_state_id(abstract_id),
          split_last_state(split_last_state) {}

    friend std::ostream &operator<<(std::ostream &os, const LegacyFlaw &flaw) {
        return os << "Flaw(" << flaw.flaw_search_state << ", "
                  << flaw.abstract_state_id << ")[split_last_state="
                  << flaw.split_last_state << "]";
    }
    bool operator==(const LegacyFlaw &other) const {
        return flaw_search_state.get_cartesian_set() == other.flaw_search_state.get_cartesian_set() &&
               abstract_state_id == other.abstract_state_id &&
               split_last_state == other.split_last_state;
    }
};

class SplitsCache {
public:
    SplitsCache();
};

class FlawSearch {
    friend class SplitsCache;

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
    bool current_bidirectional_dir_backward = false;
    bool batch_bidirectional_already_changed_dir = false;
    // {AbstractState ID -> {bw_direction -> {split_unwanted_values -> {LegacyFlaw -> {SplitAndAbsState}}}}}
    HashMap<int,
            HashMap<bool,
                    HashMap<bool,
                            HashMap<LegacyFlaw, SplitAndAbsState>>>> splits_cache;
    // If optimal transitions are different the cached split must be recomputed.
    // {AbstractState ID -> {bw_direction -> OptimalTransitions}}
    HashMap<int, HashMap<bool, OptimalTransitions>> opt_tr_cache;

    // Statistics
    int num_searches;
    int num_overall_expanded_concrete_states;
    int max_expanded_concrete_states;
    utils::Timer flaw_search_timer;
    utils::Timer compute_splits_timer;
    utils::Timer pick_split_timer;

    static void get_deviation_splits(
        const AbstractState &abs_state,
        const std::vector<State> &conc_states,
        const std::vector<int> &unaffected_variables,
        const AbstractState &target_abs_state,
        const std::vector<int> &domain_sizes,
        std::vector<std::vector<Split>> &splits,
        bool split_unwanted_values);

    static void get_deviation_splits(
        const AbstractState &abs_state,
        const std::vector<AbstractState> &flaw_search_states,
        const std::vector<int> &unaffected_variables,
        const AbstractState &target_abs_state,
        const std::vector<int> &domain_sizes,
        const int op_cost,
        std::vector<std::vector<Split>> &splits,
        bool split_unwanted_values);


    static void get_deviation_backward_splits(
        const AbstractState &abs_state,
        const std::vector<AbstractState> &flaw_search_states,
        const std::vector<int> &unaffected_variables,
        const AbstractState &source_abs_state,
        const std::vector<int> &domain_sizes,
        const int op_cost,
        std::vector<std::vector<Split>> &splits,
        bool split_unwanted_values);

    int get_abstract_state_id(const State &state) const;
    Cost get_h_value(int abstract_state_id) const;
    void add_flaw(int abs_id, const State &state);
    OptimalTransitions get_f_optimal_transitions(int abstract_state_id, bool reverse = false) const;
    OptimalTransitions get_f_optimal_backward_transitions(int abstract_state_id, bool reverse = false) const;

    void initialize();
    SearchStatus step();
    SearchStatus search_for_flaws(const utils::CountdownTimer &cegar_timer);

    std::unique_ptr<Split> create_split(
        const std::vector<StateID> &state_ids, int abstract_state_id, bool split_unwanted_values);
    std::unique_ptr<Split> create_split_from_goal_state(
        const std::vector<StateID> &state_ids, int abstract_state_id, bool split_unwanted_values);

    SplitAndAbsState create_split(
        const std::vector<AbstractState> &states, int abstract_state_id, bool split_unwanted_values);
    SplitAndAbsState create_split_from_goal_state(
        const std::vector<AbstractState> &states, int abstract_state_id, bool split_unwanted_values);
    SplitAndAbsState create_backward_split(
        const std::vector<AbstractState> &states, int abstract_state_id, bool split_unwanted_values);
    SplitAndAbsState create_backward_split_from_init_state(
        const std::vector<AbstractState> &states, int abstract_state_id, bool split_unwanted_values);

    FlawedState get_flawed_state_with_min_h();
    std::unique_ptr<Split> get_single_split(const utils::CountdownTimer &cegar_timer);
    std::unique_ptr<Split> get_min_h_batch_split(const utils::CountdownTimer &cegar_timer);

    std::vector<LegacyFlaw> get_forward_flaws(const Solution &solution,
                                              const bool in_sequence,
                                              const bool only_in_abstraction);
    std::vector<LegacyFlaw> get_backward_flaws(const Solution &solution,
                                               const bool in_sequence,
                                               const bool only_in_abstraction);

    // Return concrete state id and abstract state id where create the split.
    std::unique_ptr<LegacyFlaw> get_flaw_legacy_forward(const Solution &solution);
    // Return flaw-search state id and abstract state id where create the split.
    std::unique_ptr<LegacyFlaw> get_flaw_legacy_backward(const Solution &solution);

    SplitAndAbsState select_flaw_and_pick_split(
        std::vector<LegacyFlaw> &&flaws,
        bool backward_direction,
        utils::RandomNumberGenerator &rng);
    SplitProperties select_from_sequence_flaws(
        std::vector<LegacyFlaw> &&forward_flaws,
        std::vector<LegacyFlaw> &&backward_flaws,
        utils::RandomNumberGenerator &rng);
    SplitProperties pick_sequence_split(
        std::vector<LegacyFlaw> &&forward_flaws,
        std::vector<LegacyFlaw> &&backward_flaws,
        utils::RandomNumberGenerator &rng);

    SplitAndAbsState splits_cache_get(LegacyFlaw f, bool backward_direction, bool split_unwanted_values);

    void splits_cache_invalidate(int abstract_state_id);

    SplitAndAbsState get_split_from_flaw(const LegacyFlaw &flaw,
                                         const bool backward,
                                         const bool split_unwanted_values);

    SplitProperties get_split(const utils::CountdownTimer &cegar_timer);
    SplitProperties get_split_legacy(const Solution &solution,
                                     const bool backward = false,
                                     const bool split_unwanted_values = false);
    SplitProperties get_split_legacy_closest_to_goal(const Solution &solution,
                                                     const bool split_unwanted_values);
    void update_current_direction(const bool half_limits_reached);

public:
    FlawSearch(
        const std::shared_ptr<AbstractTask> &task,
        const Abstraction &abstraction,
        const ShortestPaths &shortest_paths,
        utils::RandomNumberGenerator &rng,
        PickFlawedAbstractState pick_flawed_abstract_state,
        PickSplit pick_split,
        PickSplit tiebreak_split,
        PickSplit sequence_split,
        int max_concrete_states_per_abstract_state,
        int max_state_expansions,
        bool intersect_flaw_search_abstract_states,
        const utils::LogProxy &log);

    SplitProperties get_split_and_direction(const Solution &solution,
                                            const utils::CountdownTimer &cegar_timer,
                                            const bool half_limits_reached);
    SplitProperties get_sequence_splits(const Solution &solution,
                                        const bool only_in_abstraction,
                                        const bool forward,
                                        const bool backward);
    bool refine_init_state() const;
    bool refine_goals() const;

    static void add_split(std::vector<std::vector<Split>> &splits, Split &&new_split,
                          bool split_unwanted_values = false);

    static std::vector<int> get_unaffected_variables(
        const OperatorProxy &op, int num_variables);

    void print_statistics() const;
};
}

namespace utils {
inline void feed(HashState &hash_state, const cegar::LegacyFlaw &val) {
    feed(hash_state, val.abstract_state_id);
    feed(hash_state, val.flaw_search_state.get_cartesian_set());
    feed(hash_state, val.split_last_state);
}
}
#endif
