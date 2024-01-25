#include "cegar_symbolic_comparison.h"

#include "../cartesian_abstractions/additive_cartesian_heuristic.h"
#include "../cartesian_abstractions/cartesian_heuristic_function.h"
#include "../cartesian_abstractions/cost_saturation.h"
#include "../cartesian_abstractions/types.h"
#include "../cartesian_abstractions/utils.h"

#include "../plugins/options.h"
#include "../plugins/plugin.h"

#include "../symbolic/search_algorithms/symbolic_search.h"

#include "../utils/logging.h"
#include "../utils/rng.h"
#include "../utils/rng_options.h"
#include "symbolic_comparing_cost_saturation.h"


using namespace std;
using namespace cartesian_abstractions;

namespace cegar_symbolic_comparison {
vector < CartesianHeuristicFunction >
CegarSymbolicComparison::generate_heuristic_functions(const plugins::Options &opts,
                                                      utils::LogProxy &log) {
    if (log.is_at_least_normal()) {
        log << "Initializing cegar-symbolic comparison..." << endl;
    }
    shared_ptr < AbstractTask > root_task = opts.get < shared_ptr < AbstractTask >> ("transform");
    shared_ptr < symbolic::SymVariables > vars = make_shared < symbolic::SymVariables > (opts, task);
    vars->init();
    vector < shared_ptr < SubtaskGenerator >> subtask_generators =
        opts.get_list < shared_ptr < SubtaskGenerator >> ("subtasks");
    shared_ptr < utils::RandomNumberGenerator > rng =
        utils::parse_rng_from_options(opts);
    CegarSymbolicComparingCostSaturation cost_saturation(
        subtask_generators, opts.get < int > ("max_states"),
        opts.get < int > ("max_transitions"), opts.get < double > ("max_abstractions_time"),
        opts.get < bool > ("use_general_costs"),
        opts.get < PickFlawedAbstractState > ("pick_flawed_abstract_state"),
        opts.get < PickSplit > ("pick_split"), opts.get < PickSplit > ("tiebreak_split"),
        opts.get < int > ("max_concrete_states_per_abstract_state"),
        opts.get < int > ("max_state_expansions"),
        opts.get < int > ("memory_padding"), *rng, log,
        opts.get < DotGraphVerbosity > ("dot_graph_verbosity"),
        opts,
        vars);
    return cost_saturation.generate_heuristic_functions(root_task);
}

CegarSymbolicComparison::CegarSymbolicComparison(
    const plugins::Options &opts)
    : AdditiveCartesianHeuristic(opts) {
    initialize(opts);
}

class CegarSymbolicComparisonFeature : public plugins::TypedFeature<Evaluator, CegarSymbolicComparison> {
public:
    CegarSymbolicComparisonFeature() : TypedFeature("cegar_symbolic_comparison") {
        document_synopsis("Comparison of symbolic backward search heuristic with CEGAR in all CEGAR subtasks");

        add_common_cegar_options(*this);
        add_option < bool > (
            "use_general_costs",
            "allow negative costs in cost partitioning",
            "true");

        Heuristic::add_options_to_feature(*this);
        symbolic::SymbolicSearch::add_options_to_feature(*this);
        add_option < std::shared_ptr < symbolic::PlanSelector >> (
            "plan_selection", "plan selection strategy", "top_k(num_plans=1)");
    }
};

static plugins::FeaturePlugin<CegarSymbolicComparisonFeature> _plugin;
}     // namespace cegar_symbolic_comparison
