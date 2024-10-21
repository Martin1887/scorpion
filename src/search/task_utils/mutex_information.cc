#include "mutex_information.h"

#include "../cartesian_abstractions/utils.h"
#include "../tasks/domain_abstracted_task.h"
#include "../utils/collections.h"
#include "parallel_hashmap/phmap.h"

using namespace std;


bool MutexInformation::are_facts_mutex(const FactPair &fact1, const FactPair &fact2) const {
    if (fact1.var == fact2.var) {
        // Same variable: mutex iff different value.
        return fact1.value != fact2.value;
    }
    assert(utils::in_bounds(fact1.var, mutexes));
    assert(utils::in_bounds(fact1.value, mutexes[fact1.var]));
    return bool(mutexes[fact1.var][fact1.value].count(fact2));
}

const std::vector<int> &MutexInformation::get_var_mutex_vars(const int var) const {
    return var_mutex_vars[var];
}

const vars_pair_queue &MutexInformation::get_mutex_vars_queue() const {
    return mutex_vars_queue;
}

const mutex_set_for_value &MutexInformation::get_var_mutexes(const int var) const {
    return var_mutex_set[var];
}

void MutexInformation::add_mutex(const FactPair &a, const FactPair &b) {
    mutexes[a.var][a.value].insert(b);
    mutexes[b.var][b.value].insert(a);
}

void MutexInformation::remove_mutex(const FactPair &a, const FactPair &b) {
    mutexes[a.var][a.value].erase(b);
    mutexes[b.var][b.value].erase(a);
}

MutexInformation MutexInformation::convert(const vector<int> &domain_size, const extra_tasks::ValueMap &value_map) const {
    int n_vars = mutexes.size();
    vector<vector<set<FactPair>>> converted_mutexes(n_vars);

    for (int var = 0; var < n_vars; var++) {
        int old_domain_size = mutexes[var].size();
        vector<set<FactPair>> var_sets(domain_size[var], set<FactPair>{});
        for (int new_value = 0; new_value < domain_size[var]; new_value++) {
            // For grouped values, mutexes can only be added if all grouped
            // values are mutex for the value.
            set<FactPair> group_mutex{};
            bool first_grouped_value = true;
            for (int old_value = 0; old_value < old_domain_size; old_value++) {
                // Old values are collected in group mutex to save only the
                // intersection for all grouped values, and they are converted
                // to the new value at the end.
                if (value_map.convert({var, old_value}).value == new_value) {
                    if (first_grouped_value) {
                        for (const FactPair &old_mutex : mutexes[var][old_value]) {
                            group_mutex.insert(old_mutex);
                        }
                        first_grouped_value = false;
                    } else {
                        vector<FactPair> to_remove{};
                        for (const FactPair &mutex : group_mutex) {
                            if (!mutexes[var][old_value].contains(mutex)) {
                                to_remove.push_back(mutex);
                            }
                        }
                        for (const FactPair &mutex : to_remove) {
                            group_mutex.erase(mutex);
                        }
                    }
                }
            }
            // The mutex can only be added as a mutex if actually all grouped
            // values are mutex.
            set<FactPair> value_converted_mutexes{};
            for (const FactPair &old_mutex : group_mutex) {
                const FactPair converted_fact = value_map.convert(old_mutex);
                int old_mutex_domain_size = mutexes[converted_fact.var].size();
                bool insert = true;
                for (int old_value = 0; old_value < old_mutex_domain_size; old_value++) {
                    if (old_value != old_mutex.value) {
                        if (value_map.convert({converted_fact.var, old_value}).value == converted_fact.value) {
                            if (!group_mutex.contains({converted_fact.var, old_value})) {
                                insert = false;
                                break;
                            }
                        }
                    }
                }
                if (insert) {
                    value_converted_mutexes.insert(value_map.convert(old_mutex));
                }
            }
            converted_mutexes[var].push_back(move(value_converted_mutexes));
        }
    }

    return MutexInformation(move(converted_mutexes));
}
