#include "disambiguation_method.h"

#include "../plugins/plugin.h"

namespace disambiguation {
CartesianState DisambiguationMethod::disambiguate_copy(const CartesianState &partial_state,
                                                       const MutexInformation &mutexes) const {
    CartesianState copy = partial_state;
    disambiguate(copy, mutexes);
    return copy;
}


class NoDisambiguationFeature : public plugins::TypedFeature<DisambiguationMethod, NoDisambiguation> {
public:
    NoDisambiguationFeature() : TypedFeature("none") {
        document_title("no disambiguation");
    }
};
static plugins::FeaturePlugin<NoDisambiguationFeature> _plugin_no_disambiguation;

static class DisambiguationMethodCategoryPlugin : public plugins::TypedCategoryPlugin<DisambiguationMethod> {
public:
    DisambiguationMethodCategoryPlugin() : TypedCategoryPlugin("DisambiguationMethod") {
        document_synopsis(
            "This page describes the various disambiguation methods supported by the planner."
            );
    }
}
_category_plugin;
}
