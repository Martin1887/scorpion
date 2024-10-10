#include "mutex_information.h"

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

const mutex_set_for_value &MutexInformation::get_var_mutexes(const int var) {
    if (!var_mutex_set.contains(var)) {
        vector<set<FactPair>> vec = mutexes[var];
        mutex_set_for_value mutex_set{};
        int size = vec.size();
        for (int value = 0; value < size; value++) {
            for (const FactPair &mutex : vec[value]) {
                mutex_set.insert({value, mutex});
            }
        }
        var_mutex_set.insert({var, std::move(mutex_set)});
    }
    return var_mutex_set.at(var);
}

void MutexInformation::add_mutex(const FactPair &a, const FactPair &b) {
    mutexes[a.var][a.value].insert(b);
    mutexes[b.var][b.value].insert(a);
}

void MutexInformation::remove_mutex(const FactPair &a, const FactPair &b) {
    mutexes[a.var][a.value].erase(b);
    mutexes[b.var][b.value].erase(a);
}
