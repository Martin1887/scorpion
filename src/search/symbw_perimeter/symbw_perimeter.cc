#include "../cegar_symbolic_comparison/symbolic_uniform_backwards_search_heuristic.h"
#include "../symbolic/original_state_space.h"
#include "../symbolic/searches/uniform_cost_search.h"

using namespace cegar_symbolic_comparison;
using namespace std;

namespace symbd_perimeter {
static shared_ptr<Heuristic> _parse(OptionParser &parser) {
    parser.add_option<double> (
        "symbw_time",
        "Time in seconds for the symbolic backward perimeter",
        "1200.0",
        Bounds("0.0", "infinity"));
    Heuristic::add_options_to_parser(parser);
    symbolic::SymbolicSearch::add_options_to_parser(parser);
    parser.add_option<std::shared_ptr<symbolic::PlanSelector>>(
        "plan_selection", "plan selection strategy", "top_k(num_plans=1)");

    Options opts = parser.parse();

    if (parser.dry_run())
        return nullptr;

    shared_ptr<SymUniformBackSearchHeuristic> h = make_shared<SymUniformBackSearchHeuristic>(opts);
    h->initialize_from_parser(opts);
    return h;
}

static Plugin<Evaluator> _plugin("symbw_perimeter", _parse);
}
