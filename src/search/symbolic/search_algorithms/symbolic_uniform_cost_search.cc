#include "symbolic_uniform_cost_search.h"
#include "../original_state_space.h"
#include "../plugins/plugin.h"
#include "../searches/bidirectional_search.h"
#include "../searches/uniform_cost_search.h"

#include "../../plugins/options.h"

#include <memory>

using namespace std;

namespace symbolic {
void SymbolicUniformCostSearch::initialize() {
    SymbolicSearch::initialize();
    mgr = make_shared < OriginalStateSpace > (vars.get(), mgrParams, search_task);

    unique_ptr < UniformCostSearch > fw_search = nullptr;
    unique_ptr < UniformCostSearch > bw_search = nullptr;

    if (fw) {
        fw_search = unique_ptr < UniformCostSearch > (
            new UniformCostSearch(this, searchParams));
    }

    if (bw) {
        bw_search = unique_ptr < UniformCostSearch > (
            new UniformCostSearch(this, searchParams));
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
                            single_solution,
                            simple);

    if (fw && bw) {
        search = unique_ptr < BidirectionalSearch > (new BidirectionalSearch(
                                                         this, searchParams, move(fw_search), move(bw_search)));
    } else {
        search.reset(fw ? fw_search.release() : bw_search.release());
    }
}

SymbolicUniformCostSearch::SymbolicUniformCostSearch(
    const plugins::Options &opts,
    const shared_ptr < AbstractTask > task)
    : SymbolicSearch(opts, task) {
    fw = opts.get<bool>("fw");
    bw = opts.get<bool>("bw");
}

void SymbolicUniformCostSearch::new_solution(const SymSolutionCut &sol) {
    if (!solution_registry->found_all_plans() && sol.get_f() < upper_bound) {
        solution_registry->register_solution(sol);
        upper_bound = sol.get_f();
    }
}
} // namespace symbolic

class SymbolicUniformCostSearchFeature : public plugins::TypedFeature<SearchAlgorithm, symbolic::SymbolicUniformCostSearch> {
public:
    SymbolicUniformCostSearchFeature() : TypedFeature("sym-uniform") {
        document_synopsis("Symbolic Uniform Cost Search");
        symbolic::SymbolicSearch::add_options_to_feature(*this);
        add_option < shared_ptr < symbolic::PlanSelector >> (
            "plan_selection", "plan selection strategy", "top_k(num_plans=1)");
        add_option<bool>("fw", "Search in the forward direction", "false");
        add_option<bool>("bw", "Search in the backward direction", "false");
    }
};
