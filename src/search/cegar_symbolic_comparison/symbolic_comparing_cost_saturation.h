#ifndef CEGAR_SYMBOLIC_COMPARING_COST_SATURATION_H
#define CEGAR_SYMBOLIC_COMPARING_COST_SATURATION_H

#include "../cegar/cartesian_heuristic_function.h"
#include "../cegar/cost_saturation.h"
#include "../cegar/flaw_search.h"
#include "../cegar/refinement_hierarchy.h"
#include "../cegar/split_selector.h"

#include "../options/options.h"

#include "../symbolic/sym_variables.h"

#include <memory>
#include <vector>

namespace cegar_symbolic_comparison {
    class CegarSymbolicComparingCostSaturation: public cegar::CostSaturation {
        const options::Options opts;
        std::shared_ptr < symbolic::SymVariables > vars;

        void build_abstractions(
            const std::vector < std::shared_ptr < AbstractTask >> &subtasks,
            const utils::CountdownTimer &timer,
            const std::function < bool() > &should_abort) override;

public:
        CegarSymbolicComparingCostSaturation(
            const std::vector < std::shared_ptr < cegar::SubtaskGenerator >> &subtask_generators,
            int max_states,
            int max_non_looping_transitions,
            double max_time,
            bool use_general_costs,
            cegar::PickFlawedAbstractState pick_flawed_abstract_state,
            cegar::PickSplit pick_split,
            cegar::PickSplit tiebreak_split,
            int max_concrete_states_per_abstract_state,
            int max_state_expansions,
            cegar::SearchStrategy search_strategy,
            int memory_padding_mb,
            utils::RandomNumberGenerator &rng,
            utils::LogProxy &log,
            cegar::DotGraphVerbosity dot_graph_verbosity,
            const options::Options &opts,
            std::shared_ptr < symbolic::SymVariables > vars);
    };
}

#endif
