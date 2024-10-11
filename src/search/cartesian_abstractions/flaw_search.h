#ifndef CARTESIAN_ABSTRACTIONS_FLAW_SEARCH_H
#define CARTESIAN_ABSTRACTIONS_FLAW_SEARCH_H

#include "abstract_state.h"
#include "flaw.h"
#include "split_selector.h"
#include "types.h"

// Needed for SearchStatus enum.
#include "../search_algorithm.h"
#include "../task_utils/cartesian_set.h"
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

namespace disambiguation {
class DisambiguationMethod;
}

namespace cartesian_abstractions {
class Abstraction;
class ShortestPaths;

// Variants from ICAPS 2022 paper (in order): FIRST, MIN_H, MAX_H, MIN_H, BATCH_MIN_H.
// See bottom of .cc file for documentation.
enum class PickFlawedAbstractState {
    FIRST,
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
    MIN_H,
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
    SEQUENCE_IN_ABSTRACTION_BIDIRECTIONAL,
    // Sequence flaws in the forward direction iteratively in abstraction from
    // the goals, starting at the initial state when no one is found.
    SEQUENCE_ITERATIVE_IN_ABSTRACTION,
    // Sequence flaws in the backward direction iteratively in abstraction from
    // the goals, starting at the initial state when no one is found.
    SEQUENCE_ITERATIVE_IN_ABSTRACTION_BACKWARD,
    // Sequence flaws in both directions iteratively in abstraction from
    // the goals, starting at the initial state when no one is found.
    SEQUENCE_ITERATIVE_IN_ABSTRACTION_BIDIRECTIONAL,
    // Sequence in batch (all progression sequence flaws are refined
    // before the next flaw search).
    SEQUENCE_BATCH,
    // Sequence in batch backward (all regression sequence flaws are refined
    // before the next flaw search).
    SEQUENCE_BATCH_BACKWARD,
    // Sequence in abstraction in batch (all progression sequence flaws are refined
    // before the next flaw search).
    SEQUENCE_IN_ABSTRACTION_BATCH,
    // Sequence in abstraction in batch backward (all regression sequence flaws are refined
    // before the next flaw search).
    SEQUENCE_IN_ABSTRACTION_BATCH_BACKWARD,
};

enum class InAbstractionFlawSearchKind {
    FALSE,
    TRUE,
    ITERATIVE_IN_REGRESSION,
};

using OptimalTransitions = phmap::flat_hash_map<int, std::vector<int>>;

struct LegacyFlaw {
    CartesianState flaw_search_state;
    int abstract_state_id;
    bool split_last_state;

    LegacyFlaw(CartesianState flaw_search_state,
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

class FlawSearch {
    TaskProxy task_proxy;
    const std::vector<int> domain_sizes;
    const Abstraction &abstraction;
    ShortestPaths &shortest_paths;
    const SplitSelector split_selector;
    utils::RandomNumberGenerator &rng;
    const PickFlawedAbstractState pick_flawed_abstract_state;
    const int max_concrete_states_per_abstract_state;
    const int max_state_expansions;
    // Intersect flaw search states with the mapped one to find more flaws.
    const bool intersect_flaw_search_abstract_states;
    std::shared_ptr<disambiguation::DisambiguationMethod> flaw_search_states_disambiguation;
    mutable utils::LogProxy log;
    mutable utils::LogProxy silent_log;  // For concrete search space.

    static const int MISSING = -1;
    constexpr static const double EPSILON = 0.000001;

    // Search data
    std::stack<StateID> open_list;
    std::unique_ptr<StateRegistry> state_registry;
    std::unique_ptr<SearchSpace> search_space;
    std::unique_ptr<PerStateInformation<int>> cached_abstract_state_ids;

    // Flaw data
    // Flaws obtained in the last flaws search. These flaws are all in the
    // same direction because interferences would exist if flaws in different
    // directions were mixed.
    std::deque<LegacyFlaw> sequence_flaws_queue;
    FlawedState last_refined_flawed_state;
    Cost best_flaw_h;
    FlawedStates flawed_states;
    bool legacy_flaws = false;
    bool in_sequence = false;
    bool in_batch = false;
    InAbstractionFlawSearchKind only_in_abstraction = InAbstractionFlawSearchKind::FALSE;
    bool forward_direction = false;
    bool backward_direction = false;
    bool split_unwanted_values = false;
    bool batch_bidirectional_already_changed_dir = false;
    // {AbstractState ID -> {bw_direction -> {split_unwanted_values -> {LegacyFlaw -> {std::shared_ptr<Split>}}}}}
    HashMap<int,
            HashMap<bool,
                    HashMap<bool,
                            HashMap<LegacyFlaw, std::shared_ptr<Split>>>>> splits_cache;
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
        const AbstractState &target_abs_state,
        const std::vector<int> &domain_sizes,
        const disambiguation::DisambiguatedOperator &op,
        std::vector<std::vector<Split>> &splits,
        bool split_unwanted_values);

    static void get_deviation_splits(
        const AbstractState &abs_state,
        const std::vector<std::reference_wrapper<const CartesianState>> &flaw_search_states,
        const AbstractState &target_abs_state,
        const std::vector<int> &domain_sizes,
        const disambiguation::DisambiguatedOperator &op,
        std::vector<std::vector<Split>> &splits,
        bool split_unwanted_values,
        bool backward = false);

    std::tuple<CartesianState, int> first_flaw_search_state(const Solution &solution,
                                                            InAbstractionFlawSearchKind only_in_abstraction,
                                                            const AbstractState * &abstract_state);

    int get_abstract_state_id(const State &state) const;
    Cost get_h_value(int abstract_state_id) const;
    void add_flaw(int abs_id, const State &state);
    OptimalTransitions get_f_optimal_transitions(int abstract_state_id) const;
    OptimalTransitions get_f_optimal_incoming_transitions(int abstract_state_id) const;
    OptimalTransitions get_f_optimal_backward_transitions(int abstract_state_id) const;

    void initialize();
    SearchStatus step();
    SearchStatus search_for_flaws(const utils::CountdownTimer &cegar_timer);

    std::unique_ptr<Split> create_split(
        const std::vector<StateID> &state_ids, int abstract_state_id, Cost solution_cost, bool split_unwanted_values);
    std::unique_ptr<Split> create_split_from_goal_state(
        const std::vector<StateID> &state_ids, int abstract_state_id, Cost solution_cost, bool split_unwanted_values);

    std::unique_ptr<Split> create_split(
        const std::vector<std::reference_wrapper<const CartesianState>> &states, int abstract_state_id, Cost solution_cost, bool split_unwanted_values);
    std::unique_ptr<Split> create_split_from_goal_state(
        const std::vector<std::reference_wrapper<const CartesianState>> &states, int abstract_state_id, Cost solution_cost, bool split_unwanted_values);
    std::unique_ptr<Split> create_backward_split(
        const std::vector<std::reference_wrapper<const CartesianState>> &states, int abstract_state_id, Cost solution_cost, bool split_unwanted_values);
    std::unique_ptr<Split> create_backward_split_from_init_state(
        const std::vector<std::reference_wrapper<const CartesianState>> &states, int abstract_state_id, Cost solution_cost, bool split_unwanted_values);

    FlawedState get_flawed_state_with_min_h();
    std::unique_ptr<Split> get_single_split(const utils::CountdownTimer &cegar_timer, Cost solution_cost);
    std::unique_ptr<Split> get_min_h_batch_split(const utils::CountdownTimer &cegar_timer, Cost solution_cost);

    // only_in_abstraction may change in the call when no flaws are found
    // with the actual value.
    std::vector<LegacyFlaw> get_forward_flaws(const Solution &solution,
                                              const InAbstractionFlawSearchKind only_in_abstraction);
    std::vector<LegacyFlaw> get_backward_flaws(const Solution &solution,
                                               const InAbstractionFlawSearchKind only_in_abstraction);

    // Return concrete state id and abstract state id where create the split.
    std::unique_ptr<LegacyFlaw> get_flaw_legacy_forward(const Solution &solution);
    // Return flaw-search state id and abstract state id where create the split.
    std::unique_ptr<LegacyFlaw> get_flaw_legacy_backward(const Solution &solution);

    // The direction here is needed for bidirectional cases.
    void push_flaw_if_not_filtered(std::vector<LegacyFlaw> &flaws,
                                   const LegacyFlaw &flaw,
                                   const Solution &solution,
                                   const bool backward_direction,
                                   std::unique_ptr<LegacyFlaw> &first_filtered_flaw,
                                   bool force_push = false);
    std::unique_ptr<Split> last_not_filtered_flaw(std::vector<LegacyFlaw> &flaws,
                                                  Cost solution_cost,
                                                  const bool backward_direction);
    // The direction here is needed for bidirectional cases.
    std::unique_ptr<Split> select_flaw_and_pick_split(
        std::vector<LegacyFlaw> &&flaws,
        bool backward_direction,
        Cost solution_cost,
        utils::RandomNumberGenerator &rng);
    SplitProperties select_from_sequence_flaws(
        std::vector<LegacyFlaw> &&forward_flaws,
        std::vector<LegacyFlaw> &&backward_flaws,
        const Solution &solution,
        utils::RandomNumberGenerator &rng);
    SplitProperties pick_sequence_split(
        std::vector<LegacyFlaw> &&forward_flaws,
        std::vector<LegacyFlaw> &&backward_flaws,
        const Solution &solution,
        utils::RandomNumberGenerator &rng);
    SplitProperties sequence_splits_tiebreak(std::unique_ptr<Split> best_fw,
                                             const AbstractState &fw_abstract_state,
                                             std::unique_ptr<Split> best_bw,
                                             const AbstractState &bw_abstract_state,
                                             int n_forward,
                                             int n_backward,
                                             const Solution &solution,
                                             bool invalidate_cache = true);
    SplitProperties return_best_sequence_split(std::unique_ptr<Split> best,
                                               bool bw_dir,
                                               int n_forward,
                                               int n_backward,
                                               const Solution &solution,
                                               bool invalidate_cache = true);

    // Direction and split_unwanted_values are needed for bidirectional cases.
    std::unique_ptr<Split> splits_cache_get(const LegacyFlaw &f,
                                            Cost solution_cost,
                                            bool backward_direction,
                                            bool split_unwanted_values);
    void splits_cache_invalidate(int abstract_state_id);

    // Direction and split_unwanted_values are needed for bidirectional cases.
    std::unique_ptr<Split> create_split_from_flaw(const LegacyFlaw &flaw,
                                                  Cost solution_cost,
                                                  const bool backward,
                                                  const bool split_unwanted_values);

    // Invoke splits_cache_get when cache makes sense and create_split_from_flaw
    // otherwise.
    std::unique_ptr<Split> get_split_from_flaw(const LegacyFlaw &flaw,
                                               Cost solution_cost,
                                               const bool backward,
                                               const bool split_unwanted_values);

    SplitProperties get_split(const utils::CountdownTimer &cegar_timer, Cost solution_cost);
    SplitProperties get_split_legacy(const Solution &solution);
    SplitProperties get_split_legacy_closest_to_goal(const Solution &solution);
    double get_plan_perc(int abstract_state_id, const Solution &solution);
    void update_current_direction(const bool half_limits_reached);

public:
    FlawSearch(
        const std::shared_ptr<AbstractTask> &task,
        const Abstraction &abstraction,
        ShortestPaths &shortest_paths,
        std::shared_ptr<TransitionSystem> &simulated_transition_system,
        utils::RandomNumberGenerator &rng,
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
        std::shared_ptr<disambiguation::DisambiguationMethod> flaw_search_states_disambiguation,
        const utils::LogProxy &log);

    SplitProperties get_split_and_direction(const Solution &solution,
                                            const utils::CountdownTimer &cegar_timer,
                                            const bool half_limits_reached);
    SplitProperties get_sequence_splits(const Solution &solution);
    bool refine_goals() const;

    static void add_split(std::vector<std::vector<Split>> &splits, Split &&new_split,
                          bool split_unwanted_values = false);

    void print_statistics() const;

    static OptimalTransitions get_f_optimal_transitions(const Abstraction &abstraction,
                                                        const ShortestPaths &shortest_paths,
                                                        int abstract_state_id);
    static OptimalTransitions get_f_optimal_incoming_transitions(const Abstraction &abstraction,
                                                                 const ShortestPaths &shortest_paths,
                                                                 int abstract_state_id);
    static OptimalTransitions get_f_optimal_backward_transitions(const Abstraction &abstraction,
                                                                 const ShortestPaths &shortest_paths,
                                                                 int abstract_state_id);
};
}

namespace utils {
inline void feed(HashState &hash_state, const cartesian_abstractions::LegacyFlaw &val) {
    feed(hash_state, val.abstract_state_id);
    feed(hash_state, val.flaw_search_state.get_cartesian_set());
    feed(hash_state, val.split_last_state);
}
}
#endif
