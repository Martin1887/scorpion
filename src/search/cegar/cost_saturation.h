#ifndef CEGAR_COST_SATURATION_H
#define CEGAR_COST_SATURATION_H

#include "split_selector.h"

#include <memory>
#include <vector>

namespace utils {
class CountdownTimer;
}

namespace cegar {
// TODO: Remove.
extern int hacked_num_landmark_abstractions;

class Abstraction;
class CartesianHeuristicFunction;
class SubtaskGenerator;
class TransitionSystem;

enum class CostPartitioningType {
    SATURATED,
    SATURATED_POSTHOC,
    SATURATED_MAX,
    OPTIMAL
};

/*
  Get subtasks from SubtaskGenerators, reduce their costs by wrapping
  them in ModifiedOperatorCostsTasks, compute Abstractions, move
  RefinementHierarchies from Abstractions to
  CartesianHeuristicFunctions, allow extracting
  CartesianHeuristicFunctions into AdditiveCartesianHeuristic.
*/
class CostSaturation {
    const CostPartitioningType cost_partitioning_type;
    const std::vector<std::shared_ptr<SubtaskGenerator>> subtask_generators;
    const int max_states;
    const int max_non_looping_transitions;
    const double max_time;
    const bool use_general_costs;
    // TODO: Remove.
    const bool exclude_abstractions_with_zero_init_h;
    const PickSplit pick_split;

    /*
      TODO: Change interface to
      AbstractionGenerator::compute_next_abstraction() and let
      CostSaturation, MaxCostSaturation and OptimalCostPartitioning use
      it.
    */
    std::vector<int> remaining_costs;
    std::vector<std::unique_ptr<Abstraction>> abstractions;
    std::vector<CartesianHeuristicFunction> heuristic_functions;
    std::vector<std::shared_ptr<TransitionSystem>> transition_systems;
    int num_abstractions;
    int num_states;
    int num_non_looping_transitions;

    void reset(const TaskProxy &task_proxy);
    std::shared_ptr<AbstractTask> get_remaining_costs_task(
        std::shared_ptr<AbstractTask> &parent) const;
    bool state_is_dead_end(const State &state) const;
    void build_abstractions(
        const std::vector<std::shared_ptr<AbstractTask>> &subtasks,
        const utils::CountdownTimer &timer,
        std::function<bool()> should_abort);
    void print_statistics() const;

public:
    CostSaturation(
        CostPartitioningType cost_partitioning_type,
        std::vector<std::shared_ptr<SubtaskGenerator>> &subtask_generators,
        int max_states,
        int max_non_looping_transitions,
        double max_time,
        bool use_general_costs,
        bool exclude_abstractions_with_zero_init_h,
        PickSplit pick_split);

    void initialize(const std::shared_ptr<AbstractTask> &task);

    std::vector<std::unique_ptr<Abstraction>> extract_abstractions();
    std::vector<CartesianHeuristicFunction> extract_heuristic_functions();
    std::vector<std::shared_ptr<TransitionSystem>> extract_transition_systems();
};

extern void reduce_costs(
    std::vector<int> &remaining_costs,
    const std::vector<int> &saturated_costs);
}

#endif
