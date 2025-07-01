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
    std::vector<std::vector<bool>> pair_in_queue;
    std::deque<std::tuple<int, int>> queue;
public:
    TrackedExistingPairsDeque(int size)
        : pair_in_queue(size, std::vector<bool>(size, 0)),
          queue() {}

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

using mutex_set_for_value = std::vector<phmap::flat_hash_set<FactPair, utils::Hash<FactPair>>>;
using vars_pair_queue = TrackedExistingPairsDeque;

class MutexInformation {
    std::vector<std::vector<std::set<FactPair>>> mutexes;
    std::shared_ptr<std::vector<int>> vars_with_mutexes;
    std::vector<std::vector<int>> mutex_vars_for_var;
    vars_pair_queue mutex_vars_queue;
    std::vector<vars_pair_queue> per_var_mutex_vars_queue;
    std::vector<mutex_set_for_value> var_mutex_set{};

public:
    MutexInformation()
        : mutexes(),
          vars_with_mutexes(),
          mutex_vars_queue(0),
          per_var_mutex_vars_queue() {}
    MutexInformation(const std::vector<std::vector<std::set<FactPair>>> &_mutexes)
        : mutexes(_mutexes),
          vars_with_mutexes(std::make_shared<std::vector<int>>()),
          mutex_vars_queue(mutexes.size()),
          per_var_mutex_vars_queue(mutexes.size(), vars_pair_queue(mutexes.size())) {
        int n_vars = mutexes.size();
        vars_with_mutexes->reserve(n_vars);
        var_mutex_set.reserve(n_vars);
        mutex_vars_for_var = std::vector<std::vector<int>>(n_vars, std::vector<int>{});
        for (int i = 0; i < n_vars; i++) {
            std::set<int> mutex_vars;
            for (const std::set<FactPair> &values_mutex : mutexes[i]) {
                for (const FactPair &mutex : values_mutex) {
                    mutex_vars.insert(mutex.var);
                }
            }
            if (!mutex_vars.empty()) {
                vars_with_mutexes->push_back(i);
            }
            mutex_vars_for_var[i].reserve(mutex_vars.size());
            for (int j : mutex_vars) {
                mutex_vars_for_var[i].push_back(j);
                mutex_vars_queue.add(i, j);
                per_var_mutex_vars_queue[i].add(j, i);
            }

            const std::vector<std::set<FactPair>> &vec = mutexes[i];
            int size = vec.size();
            var_mutex_set.push_back(std::vector<phmap::flat_hash_set<FactPair, utils::Hash<FactPair>>>(size));
            for (int value = 0; value < size; value++) {
                for (const FactPair &mutex : vec[value]) {
                    var_mutex_set[i][value].insert(mutex);
                }
            }
        }
    }

    bool are_facts_mutex(const FactPair &fact1, const FactPair &fact2) const;

    const std::set<FactPair> &get_mutexes(const FactPair &fact) const {
        return mutexes[fact.var][fact.value];
    }
    const std::vector<int> &get_mutex_vars_for_var(const int var) const;
    const std::shared_ptr<std::vector<int>> &get_vars_with_mutexes() const;
    const vars_pair_queue &get_mutex_vars_queue() const;
    const vars_pair_queue &get_mutex_vars_queue_for_var(int var) const;
    const mutex_set_for_value &get_var_mutexes(const int var) const;

    void add_mutex(const FactPair &fact1, const FactPair &fact2);
    void remove_mutex(const FactPair &fact1, const FactPair &fact2);

    MutexInformation convert(const std::vector<int> &domain_size, const extra_tasks::ValueMap &value_map) const;
};


#endif
