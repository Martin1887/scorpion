#include "ac3_pervar_disambiguation.h"

#include "cartesian_set_facts_proxy_iterator.h"
#include "../plugins/plugin.h"
#include "mutex_information.h"

using namespace std;

namespace disambiguation {
bool AC3PerVarDisambiguation::disambiguate(CartesianState &partial_state,
                                           const MutexInformation &mutexes) const {
    if (partial_state.got_empty()) {
        return false;
    }
    bool changed = false;
    CartesianSet &disambiguated = partial_state.get_mutable_cartesian_set();

    int n_vars = disambiguated.get_n_vars();
    for (int var = 0; var < n_vars; var++) {
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


class AC3PerVarDisambiguationFeature : public plugins::TypedFeature<DisambiguationMethod, AC3PerVarDisambiguation> {
public:
    AC3PerVarDisambiguationFeature() : TypedFeature("PerVarAC3") {
        document_title("AC-3 per variable (weaker) disambiguation method");
    }
};
static plugins::FeaturePlugin<AC3PerVarDisambiguationFeature> _plugin_ac3;
}
