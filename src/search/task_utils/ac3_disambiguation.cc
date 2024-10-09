#include "ac3_disambiguation.h"

#include "cartesian_set_facts_proxy_iterator.h"
#include "../plugins/plugin.h"
#include "mutex_information.h"

using namespace std;

namespace disambiguation {
bool AC3Disambiguation::disambiguate(CartesianState &partial_state,
                                     MutexInformation &mutexes) const {
    bool changed = false;
    CartesianSet disambiguated = partial_state.get_cartesian_set();

    int n_vars = disambiguated.get_n_vars();
    for (int var = 0; var < n_vars; var++) {
        mutex_set_for_value var_mutexes = mutexes.get_var_mutexes(var);
        // Initially, worklist=var_mutexes, but it changes.
        mutex_set_for_value worklist = var_mutexes;
        while (!worklist.empty()) {
            auto iterator = worklist.begin();
            tuple<int, FactPair> mutex = *iterator;
            worklist.erase(iterator);
            if (arc_reduce(disambiguated, var, mutex, var_mutexes)) {
                changed = true;
                if (disambiguated.count(var) == 0) {
                    partial_state.set_cartesian_set(move(disambiguated));
                    partial_state.got_empty();
                    return changed;
                }
                add_new_mutexes(mutex, var_mutexes, worklist);
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
                                   const tuple<int, FactPair> &mutex,
                                   const mutex_set_for_value &var_mutexes) const {
    bool change = false;
    for (auto &&[x_var, x_value] : disambiguated.iter(var)) {
        bool all_mutex = true;
        int second_var = std::get<1>(mutex).var;
        for (FactPair && y : disambiguated.iter(second_var)) {
            if (!var_mutexes.contains({x_value, y})) {
                all_mutex = false;
                break;
            }
        }
        if (all_mutex) {
            disambiguated.remove(var, x_value);
            change = true;
        }
    }

    return change;
}

void AC3Disambiguation::add_new_mutexes(const tuple<int, FactPair> &removed_mutex,
                                        const mutex_set_for_value &var_mutexes,
                                        mutex_set_for_value &worklist) const {
    int value = std::get<0>(removed_mutex);
    FactPair other_fact = std::get<1>(removed_mutex);
    for (const tuple<int, FactPair> &mutex : var_mutexes) {
        FactPair new_fact = std::get<1>(mutex);
        if (new_fact.var != other_fact.var || new_fact.value != other_fact.value) {
            worklist.insert({value, new_fact});
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
