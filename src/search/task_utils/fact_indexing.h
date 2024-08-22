#ifndef TASK_UTILS_FACT_INDEXING_H
#define TASK_UTILS_FACT_INDEXING_H

#include <vector>
#include <ranges>
#include "../task_proxy.h"

class FactID {
    int index;
public:
    explicit FactID(unsigned index) : index(index) {
    }

    operator int() const {
        return index;
    }
};

class FactPairID {
    int index;
public:
    explicit FactPairID(int index)
        : index(index) {
    }

    operator int() const {
        return index;
    }
};

class FactIndexing {
    int num_facts = 0;
    // TODO: p_index should be a single vector marking the beginning for each variable.
    std::vector<std::vector<FactID>> p_index;
    std::vector<FactPair> p_index_reverse;

public:
    FactIndexing(const AbstractTask &task);

    FactID get_id(const FactPair &fact) const {
        return p_index[fact.var][fact.value];
    }

    FactPair get_fact(FactID id) const {
        return p_index_reverse[id];
    }

    int get_num_facts() const {
        return num_facts;
    }

    auto view_all_ids() {
        return std::ranges::views::iota(num_facts) |
               std::views::transform([](int id) {return FactID(id);});
    }

    auto view_all_facts() {
        return view_all_ids() |
               std::views::transform([this](auto x) {return get_fact(x);});
    }

    auto view_all_facts_with_id() {
        return view_all_ids() |
               std::views::transform([this](auto x) {return std::make_pair(x, get_fact(x));});
    }

    // Iterate over all facts of a variable.
    auto view_fact_ids_of_var(int variable) const {
        return std::ranges::views::iota(p_index[variable].size()) |
               std::views::transform([this, variable](int id)
                                     {
                                         return FactID(p_index[variable][0] + id);
                                     });
    }
    // Iterate over all facts of a variable.
    auto view_facts_of_var(int variable) const {
        return view_fact_ids_of_var(variable) |
               std::views::transform([this](auto x) {return get_fact(x);});
    }

    [[nodiscard]] inline FactPairID get_pair_id(FactID a, FactID b) const {
        return FactPairID{static_cast<int>((a * num_facts) + b)};
    }

    [[nodiscard]] inline FactPairID get_pair_id(const FactPair &a, const FactPair &b) const {
        return get_pair_id(get_id(a), get_id(b));
    }

    std::pair<FactPair, FactPair> get_fact_pair(FactPairID id) const {
        return std::make_pair(get_fact(FactID(id / num_facts)), get_fact(FactID(id % num_facts)));
    }
};


#endif
