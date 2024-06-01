#ifndef CARTESIAN_ABSTRACTIONS_SPLIT_SELECTOR_H
#define CARTESIAN_ABSTRACTIONS_SPLIT_SELECTOR_H

#include "abstract_state.h"

#include "../task_proxy.h"

#include "../utils/logging.h"
#include "abstraction.h"
#include "shortest_paths.h"

#include <memory>
#include <vector>

namespace additive_heuristic {
class AdditiveHeuristic;
}

namespace utils {
class RandomNumberGenerator;
}

namespace cartesian_abstractions {
class AbstractState;
struct Split;

// Strategies for selecting a split in case there are multiple possibilities.
enum class PickSplit {
    RANDOM,
    // Number of values that land in the state whose h-value is probably raised.
    MIN_UNWANTED,
    MAX_UNWANTED,
    // Refinement: - (remaining_values / original_domain_size)
    MIN_REFINED,
    MAX_REFINED,
    // Compare the h^add(s_0) values of the facts.
    MIN_HADD,
    MAX_HADD,
    // Position in partial ordering of causal graph.
    MIN_CG,
    MAX_CG,
    // Compute split that covers the maximum number of flaws for several concrete states.
    MAX_COVER,
    HIGHEST_COST_OPERATOR,
    LOWEST_COST_OPERATOR,
    // Random order of variables, with a different seed at each execution of the planner.
    RANDOM_VARS_ORDER,
    // The goal distance of a child is increased after simulating the refinement.
    GOAL_DISTANCE_INCREASED,
    // The cost of the optimal abstract plan is increased after simulating the refinement.
    OPTIMAL_PLAN_COST_INCREASED,
    // Balance between most refined and closest to goal with the same weight.
    BALANCE_REFINED_CLOSEST_GOAL,
};

// Strategies for selecting a sequence flaw.
// Being able to extend `PickSplit` would be great, but it is not possible.
enum class PickSequenceFlaw {
    RANDOM,
    // Number of values that land in the state whose h-value is probably raised.
    MIN_UNWANTED,
    MAX_UNWANTED,
    // Refinement: - (remaining_values / original_domain_size)
    MIN_REFINED,
    MAX_REFINED,
    // Compare the h^add(s_0) values of the facts.
    MIN_HADD,
    MAX_HADD,
    // Position in partial ordering of causal graph.
    MIN_CG,
    MAX_CG,
    // Compute split that covers the maximum number of flaws for several concrete states.
    MAX_COVER,
    HIGHEST_COST_OPERATOR,
    LOWEST_COST_OPERATOR,
    // Random order of variables, with a different seed at each execution of the planner.
    RANDOM_VARS_ORDER,
    // The goal distance of a child is increased after simulating the refinement.
    GOAL_DISTANCE_INCREASED,
    // The cost of the optimal abstract plan is increased after simulating the refinement.
    OPTIMAL_PLAN_COST_INCREASED,
    // Balance between most refined and closest to goal with the same weight.
    BALANCE_REFINED_CLOSEST_GOAL,
    // Sequence flaw strategies.
    FIRST_FLAW,
    LAST_FLAW,
    // The first one in regression, the latest one in progression.
    CLOSEST_TO_GOAL_FLAW,
};

PickSplit sequence_to_split(const PickSequenceFlaw pick);


struct Split {
    int count;
    int abstract_state_id;
    int var_id;
    int value;
    std::vector<int> values;
    int op_cost;

    Split(int abstract_state_id, int var_id, int value, std::vector<int> &&values, int count, int op_cost = -1)
        : count(count),
          abstract_state_id(abstract_state_id),
          var_id(var_id),
          value(value),
          values(move(values)),
          op_cost(op_cost) {
        assert(count >= 1);
    }

    bool combine_with(Split &&other);

    bool operator==(const Split &other) const {
        assert(var_id == other.var_id);
        if (value == other.value) {
            return values == other.values && op_cost == other.op_cost;
        } else if (values.size() == 1 && other.values.size() == 1) {
            // If we need to separate exactly two values, their order doesn't matter.
            return value == other.values[0] && other.value == values[0] && op_cost == other.op_cost;
        } else {
            return false;
        }
    }

    friend std::ostream &operator<<(std::ostream &os, const Split &s) {
        return os << "<" << s.var_id << "=" << s.value << "|" << s.values
                  << ":" << s.count <<
               (s.op_cost > -1 ? ("(" + std::to_string(s.op_cost) + ")") : "")
                  << ">";
    }
};

struct SplitProperties {
    std::unique_ptr<Split> split;
    double flawed_state_pos_plan_length_perc;
    bool backward_direction;
    int n_forward_flawed_states;
    int n_backward_flawed_states;

    SplitProperties(std::unique_ptr<Split> split,
                    double flawed_state_pos_plan_length_perc,
                    bool backward_direction,
                    int n_forward_flaws = 0,
                    int n_backward_flaws = 0)
        : split(std::move(split)),
          flawed_state_pos_plan_length_perc(flawed_state_pos_plan_length_perc),
          backward_direction(backward_direction),
          n_forward_flawed_states(n_forward_flaws),
          n_backward_flawed_states(n_backward_flaws) {}
};

/*
  Select split in case there are multiple possible splits.
*/
class SplitSelector {
    friend class FlawSearch;

    const std::shared_ptr<AbstractTask> task;
    const TaskProxy task_proxy;
    ShortestPaths &shortest_paths;
    const Abstraction &abstraction;
    std::shared_ptr<TransitionSystem> &simulated_transition_system;
    const bool debug;
    std::vector<int> vars_order;
    std::unique_ptr<additive_heuristic::AdditiveHeuristic> additive_heuristic;

    const PickSplit first_pick;
    const PickSplit tiebreak_pick;
    const PickSequenceFlaw sequence_pick;
    const PickSequenceFlaw sequence_tiebreak_pick;

    int get_num_unwanted_values(const AbstractState &state, const Split &split) const;
    double get_refinedness(const AbstractState &state, int var_id) const;
    int get_hadd_value(int var_id, int value) const;
    int get_min_hadd_value(int var_id, const std::vector<int> &values) const;
    int get_max_hadd_value(int var_id, const std::vector<int> &values) const;

    double rate_split(const AbstractState &state, const Split &split, PickSplit pick, Cost abstract_optimal_plan_cost) const;
    std::vector<Split> compute_max_cover_splits(
        std::vector<std::vector<Split>> &&splits) const;
    Split select_from_best_splits(
        const AbstractState &abstract_state,
        std::vector<Split> &&splits,
        Cost optimal_abstract_plan_cost,
        utils::RandomNumberGenerator &rng) const;
    std::vector<Split> reduce_to_best_splits(
        const AbstractState &abstract_state,
        std::vector<std::vector<Split>> &&splits,
        Cost optimal_abstract_plan_cost) const;

public:
    SplitSelector(
        const std::shared_ptr<AbstractTask> &task,
        ShortestPaths &shortest_paths,
        const Abstraction &abstraction,
        std::shared_ptr<TransitionSystem> &simulated_transition_system,
        PickSplit pick,
        PickSplit tiebreak_pick,
        PickSequenceFlaw sequence_pick,
        PickSequenceFlaw sequence_tiebreak_pick,
        bool debug);
    ~SplitSelector();

    Split pick_split(
        const AbstractState &abstract_state,
        std::vector<std::vector<Split>> &&splits,
        Cost abstract_optimal_plan_cost,
        utils::RandomNumberGenerator &rng) const;
};
}

#endif
