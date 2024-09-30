#ifndef TASK_UTILS_CARTESIAN_STATE_H
#define TASK_UTILS_CARTESIAN_STATE_H

#include "cartesian_set.h"

#include <vector>

class ConditionsProxy;
struct FactPair;
class OperatorProxy;
class State;
class TaskProxy;

using namespace cartesian_set;

namespace disambiguation {
class DisambiguatedOperator;
}

namespace cartesian_state {
/*
  Store the Cartesian set and the ID of the node in the refinement hierarchy
  for an abstract state.
*/
class CartesianState {
protected:
    int n_vars;
    CartesianSet cartesian_set;

public:
    CartesianState(CartesianSet &&cartesian_set);
    CartesianState(const std::vector<int> &domain_sizes, std::vector<FactPair> facts);

    bool domain_subsets_intersect(const CartesianState &other, int var) const;

    // Return if the variable is fully abstracted (all possible values).
    bool is_fully_abstracted(int var) const;

    // Return the size of var's abstract domain for this state.
    int count(int var) const;

    // Return a vector with the number of concrete states represented by each
    // variable.
    std::vector<int> count() const;

    bool is_spurious() const;
    bool got_empty();

    bool is_applicable(const OperatorProxy &op) const;
    bool is_applicable(const disambiguation::DisambiguatedOperator &op) const;
    bool is_applicable(const disambiguation::DisambiguatedOperator &op, const std::vector<int> &vars) const;
    bool is_applicable(const disambiguation::DisambiguatedOperator &op, int var) const;
    bool reach_with_op(const CartesianState &other, const disambiguation::DisambiguatedOperator &op) const;
    bool reach_with_op(const CartesianState &other, const disambiguation::DisambiguatedOperator &op, const std::vector<int> &vars) const;
    bool reach_with_op(const CartesianSet &other_set, const CartesianSet &pre, const disambiguation::DisambiguatedOperator &op, int var) const;
    bool reach_with_inapplicable_op(const CartesianState &other, const disambiguation::DisambiguatedOperator &op) const;
    bool reach_with_inapplicable_op(const CartesianSet &other_set, const CartesianSet &pre, const disambiguation::DisambiguatedOperator &op, int var) const;
    bool is_backward_applicable(const OperatorProxy &op) const;
    bool is_backward_applicable(const disambiguation::DisambiguatedOperator &op) const;
    bool reach_backwards_with_op(const CartesianState &other, const disambiguation::DisambiguatedOperator &op) const;
    bool reach_backwards_with_inapplicable_op(const CartesianState &other, const disambiguation::DisambiguatedOperator &op) const;
    std::vector<int> vars_not_backward_applicable(const OperatorProxy &op) const;
    std::vector<int> vars_not_backward_applicable(const disambiguation::DisambiguatedOperator &op) const;

    // Return the Cartesian set in which applying "op" can lead to this state.
    CartesianSet regress(const OperatorProxy &op) const;
    CartesianSet regress(const disambiguation::DisambiguatedOperator &op) const;
    CartesianSet progress(const OperatorProxy &op) const;
    CartesianSet progress(const disambiguation::DisambiguatedOperator &op) const;
    CartesianSet undeviate(const CartesianState &mapped) const;

    /*
      Separate the "wanted" values from the other values in the abstract domain
      and return the resulting two new Cartesian sets.
    */
    std::pair<CartesianSet, CartesianSet> split_domain(
        int var, const std::vector<int> &wanted) const;

    bool intersects(const CartesianState &other) const;
    bool intersects(const CartesianState &other, int var) const;
    bool includes(const CartesianState &other) const;
    bool includes(const State &concrete_state) const;
    bool includes(const FactPair &fact) const;
    bool includes(int var, int value) const;
    bool includes(const std::vector<FactPair> &facts) const;

    const CartesianSet &get_cartesian_set() const;
    CartesianSet clone_cartesian_set() const;
    void set_cartesian_set(CartesianSet &&other);
    CartesianState intersection(const CartesianState &other) const;

    friend std::ostream &operator<<(std::ostream &os, const CartesianState &state) {
        return os << state.cartesian_set;
    }

    // Create the initial, unrefined abstract state.
    static std::unique_ptr<CartesianState> get_trivial_abstract_state(
        const std::vector<int> &domain_sizes);
};
}

#endif
