#include "mutex_information.h"

#include "../utils/collections.h"

bool MutexInformation::are_facts_mutex(const FactPair &fact1, const FactPair &fact2) const {
    if (fact1.var == fact2.var) {
        // Same variable: mutex iff different value.
        return fact1.value != fact2.value;
    }
    assert(utils::in_bounds(fact1.var, mutexes));
    assert(utils::in_bounds(fact1.value, mutexes[fact1.var]));
    return bool(mutexes[fact1.var][fact1.value].count(fact2));
}



void MutexInformation::add_mutex(const FactPair &a, const FactPair &b) {
    mutexes[a.var][a.value].insert(b);
    mutexes[b.var][b.value].insert(a);
}
