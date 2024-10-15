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
    CartesianSet disambiguated = partial_state.get_cartesian_set();

    int n_vars = disambiguated.get_n_vars();
    for (int var = 0; var < n_vars; var++) {
        mutex_set_for_value var_mutexes = mutexes.get_var_mutexes(var);
        vector<int> var_mutex_vars = mutexes.get_var_mutex_vars(var);
        // Initially, worklist=var_mutex_vars, but it changes.
        vector<int> worklist = var_mutex_vars;
        while (!worklist.empty()) {
            auto iterator = worklist.begin();
            int mutex_var = *iterator;
            worklist.erase(iterator);
            if (arc_reduce(disambiguated, var, mutex_var, var_mutexes)) {
                changed = true;
                if (disambiguated.count(var) == 0) {
                    partial_state.set_cartesian_set(move(disambiguated));
                    partial_state.got_empty();
                    return changed;
                }
                add_new_mutexes(mutex_var, var_mutex_vars, worklist);
            }
        }
    }

    if (changed) {
        partial_state.set_cartesian_set(move(disambiguated));
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

void AC3Disambiguation::add_new_mutexes(int removed_var,
                                        const vector<int> &var_mutex_vars,
                                        vector<int> &worklist) const {
    worklist.clear();
    for (int var : var_mutex_vars) {
        if (var != removed_var) {
            worklist.push_back(var);
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
