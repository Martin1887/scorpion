#ifndef TASK_UTILS_CARTESIAN_SET_FACTS_PROXY_ITERATOR_H
#define TASK_UTILS_CARTESIAN_SET_FACTS_PROXY_ITERATOR_H

#include "../task_proxy.h"
#include "cartesian_set.h"

namespace cartesian_set {
class CartesianSetFactsProxyIterator {
private:
    const CartesianSet *cartesian_set;
    int var_id;
    int value;
    int start_var;
    int end_var;
    bool inverse;
    int var_size;

    // If this function is not called in begin(), the first returned
    // value in the iterator is always var_id=0, value=0.
    void get_next_value_set(bool init = false) {
        assert(var_id < end_var);
        if (!init) {
            next_value();
        }
        // TODO: This should make use of specialized bitset algorithms and not test every element one by one
        while (var_id < end_var &&
               ((!inverse && !cartesian_set->test(var_id, value)) ||
                (inverse && cartesian_set->test(var_id, value)))) {
            next_value();
        }
    }

    void next_value() {
        assert(cartesian_set->count(var_id) > 0);
        assert(value <= cartesian_set->var_size(var_id) - 1);
        ++value;
        if (value >= var_size) {
            ++var_id;
            var_size = cartesian_set->var_size(var_id);
            value = 0;
        }
    }
public:
    CartesianSetFactsProxyIterator(const CartesianSet *cartesian_set,
                                   int start_var,
                                   int end_var,
                                   bool inverse)
        : cartesian_set(cartesian_set),
          var_id(start_var),
          value(0),
          start_var(start_var),
          end_var(end_var),
          inverse(inverse),
          var_size(cartesian_set->var_size(var_id)) {
    }

    // With this fancy syntax we can avoid even moves of the stack-allocated
    // struct, but don't forget to use auto &&[var, value] in the caller.
    auto operator*() const -> FactPair {
        return {var_id, value};
    }

    CartesianSetFactsProxyIterator &operator++() {
        get_next_value_set();
        return *this;
    }

    bool operator==(const CartesianSetFactsProxyIterator &other) const {
        assert(cartesian_set == other.cartesian_set);
        return var_id == other.var_id && value == other.value &&
               start_var == other.start_var && end_var == other.end_var &&
               inverse == other.inverse;
    }

    bool operator!=(const CartesianSetFactsProxyIterator &other) const {
        return !(*this == other);
    }

    auto begin() {
        var_id = start_var;
        get_next_value_set(true);
        return *this;
    }

    auto end() {
        var_id = end_var;
        value = 0;
        return *this;
    }
};
}
#endif
