#include "disambiguation_method.h"

#include "cartesian_set_facts_proxy_iterator.h"
#include "../plugins/plugin.h"

namespace disambiguation {
void DisambiguationMethod::add_disambiguation_base_options(plugins::Feature &feature) {
    feature.add_option<bool>(
        "cache_disambiguations",
        "cache disambiguations by storing the removed values in the Cartesian set",
        "true");
}


std::vector<FactPair> DisambiguationMethod::get_disambiguation_removed_values(const CartesianState &cartesian_state,
                                                                              const MutexInformation &mutex_information) {
    CartesianState disambiguated = disambiguate_copy(cartesian_state, mutex_information);
    const CartesianSet &dis_set = disambiguated.get_cartesian_set();
    std::vector<FactPair> removed_values{};

    const CartesianSet &set = cartesian_state.get_cartesian_set();
    int n_vars = set.get_n_vars();
    for (int var = 0; var < n_vars; var++) {
        for (auto &&fact : set.iter(var)) {
            if (!dis_set.test(var, fact.value)) {
                removed_values.push_back(fact);
            }
        }
    }

    return removed_values;
}

std::vector<FactPair> DisambiguationMethod::disambiguation_removed_facts(CartesianState &state,
                                                                         const MutexInformation &mutex_information) {
    if (cache_disambiguations) {
        CartesianSet &cartesian_set = state.get_mutable_cartesian_set();
        cartesian_set.set_vars_with_mutexes(mutex_information.get_vars_with_mutexes());
        if (!cache.contains(cartesian_set)) {
            cache.insert_or_assign(cartesian_set, get_disambiguation_removed_values(state, mutex_information));
        }
        return cache.at(cartesian_set);
    } else {
        return get_disambiguation_removed_values(state, mutex_information);
    }
}

CartesianState DisambiguationMethod::disambiguate_copy(const CartesianState &cartesian_state,
                                                       const MutexInformation &mutexes,
                                                       std::optional<int> var) const {
    CartesianState copy = cartesian_state;
    disambiguate(copy, mutexes, var);
    return copy;
}


class NoDisambiguationFeature : public plugins::TypedFeature<DisambiguationMethod, NoDisambiguation> {
public:
    NoDisambiguationFeature() : TypedFeature("none") {
        document_title("no disambiguation");
        DisambiguationMethod::add_disambiguation_base_options(*this);
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
