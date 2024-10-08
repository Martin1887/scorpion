#ifndef TASK_UTILS_MUTEX_INFORMATION_H
#define TASK_UTILS_MUTEX_INFORMATION_H

#include "parallel_hashmap/phmap.h"
#include "../abstract_task.h"

#include <set>

using tuple_value_fact = std::tuple<int, FactPair>;
using mutex_set_for_value = phmap::flat_hash_set<tuple_value_fact, utils::Hash<tuple_value_fact>>;

class MutexInformation {
    std::vector<std::vector<std::set<FactPair>>> mutexes;
    phmap::flat_hash_map<int, mutex_set_for_value> var_mutex_set{};

public:
    MutexInformation() = default;
    MutexInformation(const std::vector<std::vector<std::set<FactPair>>> &_mutexes)
        : mutexes(_mutexes) {}

    bool are_facts_mutex(const FactPair &fact1, const FactPair &fact2) const;

    const std::set<FactPair> &get_mutexes(const FactPair &fact) const {
        return mutexes[fact.var][fact.value];
    }
    const mutex_set_for_value &get_var_mutexes(const int var);

    void add_mutex(const FactPair &fact1, const FactPair &fact2);
    void remove_mutex(const FactPair &fact1, const FactPair &fact2);
};

namespace utils {
inline void feed(HashState &hash_state, const tuple_value_fact &val) {
    feed(hash_state, std::get<0>(val));
    feed(hash_state, std::get<1>(val));
}
}

#endif
