#ifndef CARTESIAN_ABSTRACTIONS_COST_SATURATION_H
#define CARTESIAN_ABSTRACTIONS_COST_SATURATION_H

#include "flaw_search.h"
#include "refinement_hierarchy.h"
#include "split_selector.h"
#include "subtask_generators.h"

#include <memory>
#include <vector>

namespace utils {
class CountdownTimer;
class Duration;
class RandomNumberGenerator;
class LogProxy;
}

namespace disambiguation {
class DisambiguationMethod;
}

namespace cartesian_abstractions {
class CartesianHeuristicFunction;
enum class DotGraphVerbosity;
class SubtaskGenerator;

/*
  Get subtasks from SubtaskGenerators, reduce their costs by wrapping
  them in ModifiedOperatorCostsTasks, compute Abstractions, move
  RefinementHierarchies from Abstractions to
  CartesianHeuristicFunctions, allow extracting
  CartesianHeuristicFunctions into AdditiveCartesianHeuristic.
*/
class CostSaturation {
    const std::vector<std::shared_ptr<SubtaskGenerator>> subtask_generators;
    const int max_states;
    const int max_non_looping_transitions;
    const double max_time;
    const bool use_general_costs;
    const int max_concrete_states_per_abstract_state;
    const int max_state_expansions;
    const int memory_padding_mb;
    const bool print_h_distribution;
    const bool print_useless_refinements;
    lp::LPSolverType lp_solver;
    std::shared_ptr<disambiguation::DisambiguationMethod> operators_disambiguation;
    std::shared_ptr<disambiguation::DisambiguationMethod> abstract_space_disambiguation;
    std::shared_ptr<disambiguation::DisambiguationMethod> flaw_search_states_disambiguation;
    std::shared_ptr<std::vector<disambiguation::DisambiguatedOperator>> operators;
    utils::RandomNumberGenerator &rng;
    utils::LogProxy &log;
    const cartesian_abstractions::DotGraphVerbosity dot_graph_verbosity;

    std::vector<CartesianHeuristicFunction> heuristic_functions;
    std::vector<int> remaining_costs;
    int num_states;
    int num_non_looping_transitions;

    void reset(const TaskProxy &task_proxy);
    void reduce_remaining_costs(const std::vector<int> &saturated_costs);
    std::shared_ptr<AbstractTask> get_remaining_costs_task(
        std::shared_ptr<AbstractTask> &parent) const;
    bool state_is_dead_end(const State &state) const;
    void build_abstractions(
        const SharedTasks &subtasks,
        const utils::CountdownTimer &timer,
        const std::function<bool()> &should_abort);
    void print_statistics(utils::Duration init_time) const;

public:
    CostSaturation(
        const std::vector<std::shared_ptr<SubtaskGenerator>> &subtask_generators,
        int max_states,
        int max_non_looping_transitions,
        double max_time,
        bool use_general_costs,
        int max_concrete_states_per_abstract_state,
        int max_state_expansions,
        int memory_padding_mb,
        bool print_h_distribution,
        bool print_useless_refinements,
        lp::LPSolverType lp_solver,
        std::shared_ptr<disambiguation::DisambiguationMethod> operators_disambiguation,
        std::shared_ptr<disambiguation::DisambiguationMethod> abstract_space_disambiguation,
        std::shared_ptr<disambiguation::DisambiguationMethod> flaw_search_states_disambiguation,
        utils::RandomNumberGenerator &rng,
        utils::LogProxy &log,
        DotGraphVerbosity dot_graph_verbosity);

    std::vector<CartesianHeuristicFunction> generate_heuristic_functions(
        const std::shared_ptr<AbstractTask> &task);
};
}

#endif
