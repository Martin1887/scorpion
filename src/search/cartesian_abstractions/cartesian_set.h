#ifndef CARTESIAN_ABSTRACTIONS_CARTESIAN_SET_H
#define CARTESIAN_ABSTRACTIONS_CARTESIAN_SET_H

#include "../algorithms/dynamic_bitset.h"
#include "../task_proxy.h"

#include <ostream>
#include <vector>

namespace cartesian_abstractions {
using Bitset = dynamic_bitset::DynamicBitset<unsigned short>;

/*
  For each variable store a subset of its domain.

  The underlying data structure is a vector of bitsets.
*/
class CartesianSet {
    std::vector<Bitset> domain_subsets;

public:
    explicit CartesianSet(const std::vector<int> &domain_sizes);
    explicit CartesianSet(const std::vector<int> &domain_sizes, const std::vector<FactPair> &facts);

    int n_vars() const;
    int n_values(int var) const;
    void add(int var, int value);
    void set_single_value(int var, int value);
    void remove(int var, int value);
    void add_all(int var);
    void remove_all(int var);

    bool test(int var, int value) const {
        return domain_subsets[var][value];
    }

    int count(int var) const;
    std::vector<int> get_values(int var) const;
    bool intersects(const CartesianSet &other, int var) const;
    bool is_superset_of(const CartesianSet &other) const;
    bool is_equal_in_var(const CartesianSet &other, int var) const;
    void var_union(const CartesianSet &other, int var);
    void set_var_values(const CartesianSet &other, int var);

    friend std::ostream &operator<<(
        std::ostream &os, const CartesianSet &cartesian_set);
    bool operator==(const CartesianSet &other) const;

    void feed(utils::HashState &hash_state) const;
};
}
namespace utils {
inline void feed(HashState &hash_state, const cartesian_abstractions::CartesianSet &val) {
    val.feed(hash_state);
}
}
#endif
