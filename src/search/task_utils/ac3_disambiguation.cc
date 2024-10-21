#include "ac3_disambiguation.h"

#include "cartesian_set_facts_proxy_iterator.h"
#include "../plugins/plugin.h"
#include "mutex_information.h"

using namespace std;

namespace disambiguation {
bool AC3Disambiguation::disambiguate(CartesianState &partial_state,
                                     const MutexInformation &mutexes) const {
    if (partial_state.got_empty()) {
        return false;
    }
    bool changed = false;
    CartesianSet &disambiguated = partial_state.get_mutable_cartesian_set();

    vars_pair_queue worklist = mutexes.get_mutex_vars_queue();
    while (!worklist.empty()) {
        tuple<int, int> vars_pair = worklist.pop_front();
        int var = get<0>(vars_pair);
        int mutex_var = get<1>(vars_pair);
        const mutex_set_for_value &var_mutexes = mutexes.get_var_mutexes(var);
        const vector<int> &var_mutex_vars = mutexes.get_var_mutex_vars(var);
        if (arc_reduce(disambiguated, var, mutex_var, var_mutexes)) {
            changed = true;
            if (disambiguated.count(var) == 0) {
                partial_state.got_empty();
                return changed;
            }
            add_new_mutexes(var, mutex_var, var_mutex_vars, worklist);
        }
    }

    return changed;
}

bool AC3Disambiguation::arc_reduce(CartesianSet &disambiguated,
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

void AC3Disambiguation::add_new_mutexes(int current_var,
                                        int removed_var,
                                        const vector<int> &var_mutex_vars,
                                        vars_pair_queue &worklist) const {
    // (Z, X) such as there is a relation (X, Z) or (Z, X) and Z!=Y.
    for (int var : var_mutex_vars) {
        if (var != removed_var) {
            worklist.add(var, current_var);
        }
    }
}

class AC3DisambiguationFeature : public plugins::TypedFeature<DisambiguationMethod, AC3Disambiguation> {
public:
    AC3DisambiguationFeature() : TypedFeature("AC3") {
        document_title("AC-3 disambiguation method");
    }
};
static plugins::FeaturePlugin<AC3DisambiguationFeature> _plugin_ac3;
}
