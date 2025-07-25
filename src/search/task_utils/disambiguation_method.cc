#include "disambiguation_method.h"

#include "cartesian_set_facts_proxy_iterator.h"
#include "../plugins/plugin.h"

namespace disambiguation {
void DisambiguationMethod::add_disambiguation_base_options(plugins::Feature &feature) {
    feature.add_option<bool>(
        "cache_disambiguations",
        "cache disambiguations by storing the removed values in the Cartesian set",
        "false");
}


std::vector<FactPair> DisambiguationMethod::get_disambiguation_removed_values(const CartesianSet &set,
                                                                              const CartesianSet &dis_set) {
    std::vector<FactPair> removed_values{};

    int n_vars = set.get_n_vars();
    for (int var = 0; var < n_vars; var++) {
        int var_size = set.var_size(var);
        for (int value = 0; value < var_size; value++) {
            if (set.test(var, value) && !dis_set.test(var, value)) {
                removed_values.push_back({var, value});
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
            cache.insert_or_assign(cartesian_set,
                                   get_disambiguation_removed_values(cartesian_set, disambiguate_copy(state, mutex_information).get_cartesian_set()));
        }
        return cache.at(cartesian_set);
    } else {
        return get_disambiguation_removed_values(state.get_cartesian_set(), disambiguate_copy(state, mutex_information).get_cartesian_set());
    }
}

std::vector<FactPair> DisambiguationMethod::disambiguation_removed_facts(CartesianState &state,
                                                                         const MutexInformation &mutex_information,
                                                                         const std::vector<int> &modified_vars) {
    if (cache_disambiguations) {
        CartesianSet &cartesian_set = state.get_mutable_cartesian_set();
        cartesian_set.set_vars_with_mutexes(mutex_information.get_vars_with_mutexes());
        if (!cache.contains(cartesian_set)) {
            cache.insert_or_assign(cartesian_set,
                                   get_disambiguation_removed_values(cartesian_set, disambiguate_copy(state, mutex_information, modified_vars).get_cartesian_set()));
        }
        return cache.at(cartesian_set);
    } else {
        return get_disambiguation_removed_values(state.get_cartesian_set(), disambiguate_copy(state, mutex_information, modified_vars).get_cartesian_set());
    }
}

CartesianState DisambiguationMethod::disambiguate_copy(const CartesianState &cartesian_state,
                                                       const MutexInformation &mutexes,
                                                       std::optional<int> var) const {
    CartesianState copy = cartesian_state;
    disambiguate(copy, mutexes, var);
    return copy;
}

CartesianState DisambiguationMethod::disambiguate_copy(const CartesianState &cartesian_state,
                                                       const MutexInformation &mutexes,
                                                       const std::vector<int> &modified_vars) const {
    CartesianState copy = cartesian_state;
    disambiguate(copy, mutexes, modified_vars);
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
