#include "eager_osp_search.h"
#include "search_common.h"

#include "../plugins/options.h"
#include "../plugins/plugin.h"

using namespace std;

namespace plugin_eager_osp {
static shared_ptr<SearchAlgorithm> _parse(plugins::Feature &feature) {
    parser.document_synopsis("Eager best-first search", "");

    feature.add_option<shared_ptr<OpenListFactory>>("open", "open list");
    feature.add_option<bool>("reopen_closed", "reopen closed nodes", "false");
    feature.add_option<shared_ptr<Evaluator>>(
        "f_eval",
        "set evaluator for jump statistics. "
        "(Optional; if no evaluator is used, jump statistics will not be "
        "displayed.)",
        OptionParser::NONE);
    parser.add_list_option<shared_ptr<Evaluator>>(
        "preferred", "use preferred operators of these evaluators", "[]");

    SearchAlgorithm::add_pruning_option(parser);
    SearchAlgorithm::add_options_to_feature(parser);
    Options opts = parser.parse();

    shared_ptr<eager_search::EagerOspSearch> engine;
    if (!parser.dry_run()) {
        engine = make_shared<eager_search::EagerOspSearch>(opts);
    }

    return engine;
}

static Plugin<SearchAlgorithm> _plugin("eager_osp", _parse);
} // namespace plugin_eager_osp
