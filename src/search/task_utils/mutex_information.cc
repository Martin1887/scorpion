#include "mutex_information.h"

#include "../utils/collections.h"

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

const set<tuple<int, FactPair>> MutexInformation::get_var_mutexes(const int var) {
    if (!var_mutex_set.contains(var)) {
        vector<set<FactPair>> vec = mutexes[var];
        set<tuple<int, FactPair>> mutex_set{};
        for (int value = 0; value < static_cast<int>(vec.size()); value++) {
            for (FactPair mutex : vec[value]) {
                mutex_set.insert(make_tuple(value, mutex));
            }
        }
        var_mutex_set.insert({var, move(mutex_set)});
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
