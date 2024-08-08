#ifndef TASK_UTILS_CARTESIAN_SET_FACTS_PROXY_ITERATOR_H
#define TASK_UTILS_CARTESIAN_SET_FACTS_PROXY_ITERATOR_H

#include "../task_proxy.h"
#include "cartesian_set.h"

namespace cartesian_set {
class CartesianSetFactsProxyIterator {
    const CartesianSet *cartesian_set;
    int var_id;
    int value;
public:
    CartesianSetFactsProxyIterator(const CartesianSet *cartesian_set, int var_id, int value)
        : cartesian_set(cartesian_set), var_id(var_id), value(value) {}

    FactPair operator*() const {
        return {var_id, value};
    }

    CartesianSetFactsProxyIterator &operator++() {
        int num_variables = cartesian_set->n_vars();
        do {     //TODO: This should make use of specialized bitset algorithms and not test every element one by one
            assert(var_id < num_variables);
            int num_facts = cartesian_set->n_values(var_id);
            assert(value < num_facts);
            ++value;
            if (value == num_facts) {
                ++var_id;
                value = 0;
            }
        } while (!cartesian_set->test(var_id, value) && var_id < num_variables);

        return *this;
    }

    bool operator==(const CartesianSetFactsProxyIterator &other) const {
        assert(cartesian_set == other.cartesian_set);
        return var_id == other.var_id && value == other.value;
    }

    bool operator!=(const CartesianSetFactsProxyIterator &other) const {
        return !(*this == other);
    }
};
}
#endif
