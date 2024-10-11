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
    int n_vars;
    bool empty;

    void init_facts(const std::vector<FactPair> &facts);
    void init_facts(const PreconditionsProxy &facts);

public:
    explicit CartesianSet(const TaskProxy &task);
    explicit CartesianSet(const TaskProxy &task, const std::vector<FactPair> &facts);
    explicit CartesianSet(const TaskProxy &task, const PreconditionsProxy &facts);
    explicit CartesianSet(const std::vector<int> &domain_sizes);
    explicit CartesianSet(const std::vector<int> &domain_sizes, const std::vector<FactPair> &facts);
    explicit CartesianSet(const std::vector<int> &domain_sizes, const PreconditionsProxy &facts);

    int get_n_vars() const;
    void add(int var, int value);
    void set_single_value(int var, int value);
    void remove(int var, int value);
    void add_all(int var);
    void remove_all(int var);
    CartesianSet intersection(const CartesianSet &other) const;
    utils::HashSet<int> var_intersection(const CartesianSet &other, int var) const;

    bool test(int var, int value) const {
        return domain_subsets[var][value];
    }

    int count(int var) const;
    int var_size(int var) const;
    bool got_empty();
    bool is_empty() const;
    bool all_values_set(int var) const;
    std::vector<int> get_values(int var) const;
    std::vector<int> get_intersection_values(int var, const CartesianSet &other) const;
    utils::HashSet<int> get_values_set(int var) const;
    void set_values(int var, const std::vector<int> &values);
    void set_values(int var, const utils::HashSet<int> &values);
    void set_values(int var, const CartesianSet &other);
    void set_intersection_values(int var, const CartesianSet &other);
    bool intersects_intersection(const CartesianSet &other, const CartesianSet &another, int var) const;
    bool intersects(const CartesianSet &other, int var) const;
    bool intersects(const CartesianSet &other) const;
    bool is_superset_of(const CartesianSet &other) const;
    bool is_equal_in_var(const CartesianSet &other, int var) const;

    CartesianSetFactsProxyIterator iter(int start, int end, bool inverse = false) const;
    // Iterator for only the specified var.
    CartesianSetFactsProxyIterator iter(int start) const;
    // Iterator for all values.
    CartesianSetFactsProxyIterator iter() const;
    // Iterator over non matching values.
    CartesianSetFactsProxyIterator inverse_iter() const;

    friend std::ostream &operator<<(
        std::ostream &os, const CartesianSet &cartesian_set);

    bool operator==(const CartesianSet &other) const;
};
}

namespace utils {
inline void feed(HashState &hash_state, const cartesian_set::CartesianSet &val) {
    int n_vars = val.get_n_vars();
    feed(hash_state, n_vars);
    for (int var = 0; var < n_vars; var++) {
        feed(hash_state, val.get_values(var));
    }
}
}

#endif
