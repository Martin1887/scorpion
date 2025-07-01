#include "ac3_pervar_disambiguation.h"

#include "cartesian_set_facts_proxy_iterator.h"
#include "../plugins/plugin.h"
#include "mutex_information.h"

using namespace std;

namespace disambiguation {
bool AC3PerVarDisambiguation::disambiguate(CartesianState &partial_state,
                                           const MutexInformation &mutexes,
                                           optional<int> var) const {
    if (partial_state.got_empty()) {
        return false;
    }
    bool changed = false;
    CartesianSet &disambiguated = partial_state.get_mutable_cartesian_set();

    int n_vars = disambiguated.get_n_vars();
    int init_var = 0;
    int final_var = n_vars;
    if (var.has_value()) {
        init_var = var.value();
        final_var = init_var + 1;
    }
    for (int var = init_var; var < final_var; var++) {
        const mutex_set_for_value &var_mutexes = mutexes.get_var_mutexes(var);
        // Initially, worklist=var_mutex_vars, but it changes.
        vector<int> worklist = mutexes.get_var_mutex_vars(var);
        while (!worklist.empty()) {
            auto iterator = worklist.begin();
            int mutex_var = *iterator;
            worklist.erase(iterator);
            if (arc_reduce(disambiguated, var, mutex_var, var_mutexes)) {
                changed = true;
                if (disambiguated.count(var) == 0) {
                    partial_state.got_empty();
                    return changed;
                }
            }
        }
    }

    return changed;
}

bool AC3PerVarDisambiguation::arc_reduce(CartesianSet &disambiguated,
                                         int var,
                                         int mutex_var,
                                         const mutex_set_for_value &var_mutexes) const {
    bool change = false;
    int var_size = disambiguated.var_size(var);
    int mutex_var_size = disambiguated.var_size(mutex_var);
    for (int x_value = 0; x_value < var_size; x_value++) {
        if (disambiguated.test(var, x_value)) {
            bool all_mutex = true;
            for (int y_value = 0; y_value < mutex_var_size; y_value++) {
                if (disambiguated.test(mutex_var, y_value)) {
                    if (!var_mutexes.contains({x_value, {mutex_var, y_value}})) {
                        all_mutex = false;
                        break;
                    }
                }
            }
            if (all_mutex) {
                disambiguated.remove(var, x_value);
                change = true;
            }
        }
    }

    return change;
}

bool AC3PerVarDisambiguation::test_disambiguate(const CartesianState &partial_state, const MutexInformation &mutexes, int var, const std::set<int> &values_for_var) const {
    const mutex_set_for_value &var_mutexes = mutexes.get_var_mutexes(var);
    vector<int> worklist = mutexes.get_var_mutex_vars(var);
    while (!worklist.empty()) {
        auto iterator = worklist.begin();
        int mutex_var = *iterator;
        worklist.erase(iterator);
        if (test_arc_reduce(partial_state.get_cartesian_set(), values_for_var, mutex_var, var_mutexes)) {
            return true;
        }
    }

    return false;
}

bool AC3PerVarDisambiguation::test_arc_reduce(const CartesianSet &partial_state,
                                              const set<int> &values_for_var,
                                              int mutex_var,
                                              const mutex_set_for_value &var_mutexes) const {
    int mutex_var_size = partial_state.var_size(mutex_var);
    for (int x_value : values_for_var) {
        bool all_mutex = true;
        for (int y_value = 0; y_value < mutex_var_size; y_value++) {
            if (partial_state.test(mutex_var, y_value)) {
                if (!var_mutexes.contains({x_value, {mutex_var, y_value}})) {
                    all_mutex = false;
                    break;
                }
            }
        }
        if (all_mutex) {
            return true;
        }
    }

    return false;
}


class AC3PerVarDisambiguationFeature : public plugins::TypedFeature<DisambiguationMethod, AC3PerVarDisambiguation> {
public:
    AC3PerVarDisambiguationFeature() : TypedFeature("PerVarAC3") {
        document_title("AC-3 per variable (weaker) disambiguation method");
        DisambiguationMethod::add_disambiguation_base_options(*this);
    }
};
static plugins::FeaturePlugin<AC3PerVarDisambiguationFeature> _plugin_pervar_ac3;
}
