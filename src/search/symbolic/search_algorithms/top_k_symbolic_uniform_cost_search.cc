#include "top_k_symbolic_uniform_cost_search.h"

#include "../original_state_space.h"
#include "../plugins/plugin.h"
#include "../searches/bidirectional_search.h"
#include "../searches/top_k_uniform_cost_search.h"
#include "../../plugins/options.h"

#include <memory>

using namespace std;

namespace symbolic {
void TopkSymbolicUniformCostSearch::initialize() {
    SymbolicSearch::initialize();

    mgr = make_shared<OriginalStateSpace>(vars.get(), mgrParams, search_task);

    unique_ptr<TopkUniformCostSearch> fw_search = nullptr;
    unique_ptr<TopkUniformCostSearch> bw_search = nullptr;

    if (fw) {
        fw_search = unique_ptr<TopkUniformCostSearch>(
            new TopkUniformCostSearch(this, searchParams));
    }

    if (bw) {
        bw_search = unique_ptr<TopkUniformCostSearch>(
            new TopkUniformCostSearch(this, searchParams));
    }

    if (fw) {
        fw_search->init(mgr, true, bw_search.get());
    }

    if (bw) {
        bw_search->init(mgr, false, fw_search.get());
    }

    auto individual_trs = fw ? fw_search->getStateSpaceShared()->getIndividualTRs() :  bw_search->getStateSpaceShared()->getIndividualTRs();

    solution_registry->init(vars,
                            fw_search ? fw_search->getClosedShared() : nullptr,
                            bw_search ? bw_search->getClosedShared() : nullptr,
                            individual_trs,
                            plan_data_base,
                            false,
                            simple);

    if (fw && bw) {
        search = unique_ptr<BidirectionalSearch>(new BidirectionalSearch(
                                                     this, searchParams, move(fw_search), move(bw_search)));
    } else {
        search.reset(fw ? fw_search.release() : bw_search.release());
    }
}

TopkSymbolicUniformCostSearch::TopkSymbolicUniformCostSearch(
    const plugins::Options &opts)
    : SymbolicUniformCostSearch(opts) {}

void TopkSymbolicUniformCostSearch::new_solution(const SymSolutionCut &sol) {
    if (!solution_registry->found_all_plans()) {
        solution_registry->register_solution(sol);
    } else {
        lower_bound = numeric_limits<int>::max();
    }
}
}

class TopkSymbolicUniformCostSearchFeature : public plugins::TypedFeature<SearchAlgorithm, symbolic::TopkSymbolicUniformCostSearch> {
public:
    TopkSymbolicUniformCostSearchFeature() : TypedFeature("symk") {
        document_synopsis("Top-k Symbolic Bidirectional Uniform Cost Search");
        symbolic::SymbolicSearch::add_options_to_feature(*this);
        add_option<shared_ptr<symbolic::PlanSelector>>(
            "plan_selection", "plan selection strategy");
        add_option<bool>("fw", "Search in the forward direction", "false");
        add_option<bool>("bw", "Search in the backward direction", "false");
    }
};


static plugins::FeaturePlugin<TopkSymbolicUniformCostSearchFeature> _plugin;
