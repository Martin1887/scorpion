#include "cegar_symbolic_comparison.h"

#include "../cegar/additive_cartesian_heuristic.h"
#include "../cegar/cartesian_heuristic_function.h"
#include "../cegar/cost_saturation.h"
#include "../cegar/types.h"
#include "../cegar/utils.h"

#include "../option_parser.h"
#include "../plugin.h"

#include "../symbolic/search_engines/symbolic_search.h"

#include "../utils/logging.h"
#include "../utils/markup.h"
#include "../utils/rng.h"
#include "../utils/rng_options.h"
#include "symbolic_comparing_cost_saturation.h"

#include <cassert>

using namespace std;
using namespace cegar;

namespace cegar_symbolic_comparison {
    vector < CartesianHeuristicFunction >
    CegarSymbolicComparison::generate_heuristic_functions(const options::Options &opts,
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
            opts.get < int > ("max_transitions"), opts.get < double > ("max_time"),
            opts.get < bool > ("use_general_costs"),
            opts.get < PickFlawedAbstractState > ("pick_flawed_abstract_state"),
            opts.get < PickSplit > ("pick_split"), opts.get < PickSplit > ("tiebreak_split"),
            opts.get < int > ("max_concrete_states_per_abstract_state"),
            opts.get < int > ("max_state_expansions"),
            opts.get < SearchStrategy > ("search_strategy"),
            opts.get < int > ("memory_padding"), *rng, log,
            opts.get < DotGraphVerbosity > ("dot_graph_verbosity"),
            opts,
            vars);
        return cost_saturation.generate_heuristic_functions(root_task);
    }

    CegarSymbolicComparison::CegarSymbolicComparison(
        const options::Options &opts)
        : AdditiveCartesianHeuristic(opts) {
        initialize(opts);
    }

    static shared_ptr < Heuristic > _parse(OptionParser & parser) {
        parser.document_synopsis("Comparison of symbolic backward search heuristic with CEGAR in all CEGAR subtasks", "");

        add_common_cegar_options(parser);
        parser.add_option < bool > (
            "use_general_costs",
            "allow negative costs in cost partitioning",
            "true");
        Heuristic::add_options_to_parser(parser);

        symbolic::SymbolicSearch::add_options_to_parser(parser);
        parser.add_option < std::shared_ptr < symbolic::PlanDataBase >> (
            "plan_selection", "plan selection strategy", "top_k(num_plans=1)");

        Options opts = parser.parse();
        // non-stop must be true to do the symbolic searches until the end
        opts.set("non_stop", true);

        if (parser.dry_run())
            return nullptr;

        return make_shared < CegarSymbolicComparison > (opts);
    }

    static Plugin < Evaluator > _plugin("cegar_symbolic_comparison", _parse);
}     // namespace cegar_symbolic_comparison
