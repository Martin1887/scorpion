#ifndef TASK_UTILS_MUTEX_INFORMATION_H
#define TASK_UTILS_MUTEX_INFORMATION_H

#include "../abstract_task.h"

#include <set>

class MutexInformation {
    std::vector<std::vector<std::set<FactPair>>> mutexes;

public:
    MutexInformation() = default;
    MutexInformation(const std::vector<std::vector<std::set<FactPair>>> & _mutexes) : mutexes(_mutexes) {
    }
    bool are_facts_mutex(const FactPair &fact1, const FactPair &fact2) const;
    const std::set<FactPair> & get_mutexes (const FactPair &fact) const {
        return mutexes[fact.var][fact.value];
    }

    void add_mutex (const FactPair &fact1, const FactPair &fact2);
};

#endif
