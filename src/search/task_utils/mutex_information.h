#ifndef TASK_UTILS_MUTEX_INFORMATION_H
#define TASK_UTILS_MUTEX_INFORMATION_H

#include "parallel_hashmap/phmap.h"
#include "../abstract_task.h"

#include <deque>
#include <set>

namespace extra_tasks {
class ValueMap;
}

class TrackedExistingPairsDeque {
    std::deque<std::tuple<int, int>> queue;
    std::vector<std::vector<bool>> pair_in_queue;
public:
    TrackedExistingPairsDeque(int size)
        : queue(),
          pair_in_queue(size, std::vector<bool>(size, 0)) {}

    void add(int first, int second) {
        if (!pair_in_queue[first][second]) {
            queue.push_back({first, second});
            pair_in_queue[first][second] = true;
        }
    }

    std::tuple<int, int> pop_front() {
        std::tuple<int, int> pop = queue.front();
        queue.pop_front();
        pair_in_queue[std::get<0>(pop)][std::get<1>(pop)] = false;

        return pop;
    }

    bool empty() const {
        return queue.empty();
    }
};

using tuple_value_fact = std::tuple<int, FactPair>;
using mutex_set_for_value = phmap::flat_hash_set<tuple_value_fact, utils::Hash<tuple_value_fact>>;
using vars_pair_queue = TrackedExistingPairsDeque;

class MutexInformation {
    std::vector<std::vector<std::set<FactPair>>> mutexes;
    std::vector<std::vector<int>> var_mutex_vars;
    vars_pair_queue mutex_vars_queue;
    std::vector<mutex_set_for_value> var_mutex_set{};

public:
    MutexInformation()
        : mutexes(),
          mutex_vars_queue(0) {}
    MutexInformation(const std::vector<std::vector<std::set<FactPair>>> &_mutexes)
        : mutexes(_mutexes),
          mutex_vars_queue(mutexes.size()) {
        int n_vars = mutexes.size();
        var_mutex_set.reserve(n_vars);
        var_mutex_vars = std::vector<std::vector<int>>(n_vars, std::vector<int>{});
        for (int i = 0; i < n_vars; i++) {
            std::set<int> mutex_vars;
            for (const std::set<FactPair> &values_mutex : mutexes[i]) {
                for (const FactPair &mutex : values_mutex) {
                    mutex_vars.insert(mutex.var);
                }
            }
            var_mutex_vars[i].reserve(mutex_vars.size());
            for (int j : mutex_vars) {
                var_mutex_vars[i].push_back(j);
                mutex_vars_queue.add(i, j);
            }

            const std::vector<std::set<FactPair>> &vec = mutexes[i];
            mutex_set_for_value mutex_set{};
            int size = vec.size();
            for (int value = 0; value < size; value++) {
                for (const FactPair &mutex : vec[value]) {
                    mutex_set.insert({value, mutex});
                }
            }
            var_mutex_set.push_back(std::move(mutex_set));
        }
    }

    bool are_facts_mutex(const FactPair &fact1, const FactPair &fact2) const;

    const std::set<FactPair> &get_mutexes(const FactPair &fact) const {
        return mutexes[fact.var][fact.value];
    }
    const std::vector<int> &get_var_mutex_vars(const int var) const;
    const vars_pair_queue &get_mutex_vars_queue() const;
    const mutex_set_for_value &get_var_mutexes(const int var) const;

    void add_mutex(const FactPair &fact1, const FactPair &fact2);
    void remove_mutex(const FactPair &fact1, const FactPair &fact2);

    MutexInformation convert(const std::vector<int> &domain_size, const extra_tasks::ValueMap &value_map) const;
};

namespace utils {
inline void feed(HashState &hash_state, const tuple_value_fact &val) {
    feed(hash_state, std::get<0>(val));
    feed(hash_state, std::get<1>(val));
}
inline void feed(HashState &hash_state, const std::tuple<int, int> &val) {
    feed(hash_state, std::get<0>(val));
    feed(hash_state, std::get<1>(val));
}
}

#endif
