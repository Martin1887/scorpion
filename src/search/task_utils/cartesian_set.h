#ifndef TASK_UTILS_CARTESIAN_SET_H
#define TASK_UTILS_CARTESIAN_SET_H

#include "../algorithms/dynamic_bitset.h"
#include "../task_proxy.h"

#include <cmath>
#include <ostream>
#include <vector>

namespace cartesian_set {
using Bitset = dynamic_bitset::DynamicBitset<unsigned short>;

class CartesianSetFactsProxyIterator;

/*
  For each variable store a subset of its domain.

  The underlying data structure is a vector of bitsets.
*/
class CartesianSet {
    std::vector<Bitset> domain_subsets;

private:
    void init_facts(std::vector<FactPair> facts);

public:
    explicit CartesianSet(const TaskProxy &task);
    explicit CartesianSet(const TaskProxy &task, std::vector<FactPair> facts);
    explicit CartesianSet(const std::vector<int> &domain_sizes);
    explicit CartesianSet(const std::vector<int> &domain_sizes, std::vector<FactPair> facts);

    int n_vars() const;
    int n_values(int var) const;
    void add(int var, int value);
    void set_single_value(int var, int value);
    void remove(int var, int value);
    void add_all(int var);
    void remove_all(int var);
    CartesianSet intersection(const CartesianSet &other) const;

    bool test(int var, int value) const {
        return domain_subsets[var][value];
    }

    int count(int var) const;
    bool all_values_set(int var) const;
    std::vector<int> get_values(int var) const;
    bool intersects(const CartesianSet &other, int var) const;
    bool intersects(const CartesianSet &other) const;
    bool is_superset_of(const CartesianSet &other) const;

    CartesianSetFactsProxyIterator begin(int var) const;

    CartesianSetFactsProxyIterator begin() const;

    CartesianSetFactsProxyIterator end() const;

    friend std::ostream &operator<<(
        std::ostream &os, const CartesianSet &cartesian_set);

    bool operator==(const CartesianSet &other) const;
};
}

namespace utils {
inline void feed(HashState &hash_state, const cartesian_set::CartesianSet &val) {
    int n_vars = val.n_vars();
    feed(hash_state, n_vars);
    for (int var = 0; var < n_vars; var++) {
        feed(hash_state, val.get_values(var));
    }
}
}

#endif
