#ifndef CEGAR_SYMBOLIC_COMPARING_COST_SATURATION_H
#define CEGAR_SYMBOLIC_COMPARING_COST_SATURATION_H

#include "../cartesian_abstractions/cost_saturation.h"
#include "../cartesian_abstractions/flaw_search.h"
#include "../cartesian_abstractions/refinement_hierarchy.h"
#include "../cartesian_abstractions/split_selector.h"

#include "../plugins/options.h"

#include "../symbolic/sym_variables.h"

#include <memory>
#include <vector>

namespace cegar_symbolic_comparison {
class CegarSymbolicComparingCostSaturation : public cartesian_abstractions::CostSaturation {
    const plugins::Options opts;
    std::shared_ptr < symbolic::SymVariables > vars;

    void build_abstractions(
        const std::vector < std::shared_ptr < AbstractTask >> &subtasks,
        const utils::CountdownTimer &timer,
        const std::function < bool() > &should_abort) override;

public:
    CegarSymbolicComparingCostSaturation(
        const std::vector < std::shared_ptr < cartesian_abstractions::SubtaskGenerator >> &subtask_generators,
        int max_states,
        int max_non_looping_transitions,
        double max_time,
        bool use_general_costs,
        cartesian_abstractions::PickFlawedAbstractState pick_flawed_abstract_state,
        cartesian_abstractions::PickSplit pick_split,
        cartesian_abstractions::PickSplit tiebreak_split,
        int max_concrete_states_per_abstract_state,
        int max_state_expansions,
        int memory_padding_mb,
        utils::RandomNumberGenerator &rng,
        utils::LogProxy &log,
        cartesian_abstractions::DotGraphVerbosity dot_graph_verbosity,
        const plugins::Options &opts,
        std::shared_ptr < symbolic::SymVariables > vars);
};
}

#endif
