#ifndef CARTESIAN_ABSTRACTIONS_ABSTRACT_STATE_H
#define CARTESIAN_ABSTRACTIONS_ABSTRACT_STATE_H

#include "cartesian_set.h"
#include "types.h"

#include <vector>

class ConditionsProxy;
struct FactPair;
class OperatorProxy;
class State;
class TaskProxy;

namespace cartesian_abstractions {
class TransitionSystem;
/*
  Store the Cartesian set and the ID of the node in the refinement hierarchy
  for an abstract state.
*/
class AbstractState {
    int state_id;

    // This state's node in the refinement hierarchy.
    NodeID node_id;

    CartesianSet cartesian_set;

public:
    AbstractState(int state_id, NodeID node_id, CartesianSet &&cartesian_set);
    AbstractState(AbstractState &&abstract_state) = default;
    AbstractState(const AbstractState &abstract_state) = default;
    AbstractState(int state_id, NodeID node_id, const std::vector<int> &domain_sizes, const std::vector<FactPair> &facts);

    const CartesianSet &get_cartesian_set() const;
    CartesianSet clone_cartesian_set() const;

    int n_vars() const;

    bool domain_subsets_intersect(const CartesianSet &other, const std::vector<int> &vars) const;
    bool domain_subsets_intersect(const CartesianSet &other, const std::vector<bool> &vars) const;
    bool domain_subsets_intersect(const CartesianSet &other, int var) const;
    bool domain_subsets_intersect(const AbstractState &other, int var) const;

    // Return the size of var's abstract domain for this state.
    int count(int var) const;

    bool contains(int var, int value) const;

    bool is_applicable(const OperatorProxy &op) const;
    bool is_backward_applicable(const std::vector<std::unordered_set<int>> &post) const;
    bool is_backward_applicable(int var, const std::unordered_set<int> &var_post) const;
    bool reach_with_op(const AbstractState &other, const OperatorProxy &op, const TransitionSystem &ts) const;
    bool reach_backwards_with_op(const AbstractState &other, const OperatorProxy &op) const;
    // Transform the Cartesian set into the succesor.
    void progress(const OperatorProxy &op);
    // Transform the Cartesian set into the one which applying "op" can lead to this state.
    void regress(const OperatorProxy &op);
    // Inner intersection with another abstract state.
    void intersect(const AbstractState &other);
    void undeviate(const AbstractState &mapped);
    bool intersects(const AbstractState &other) const;
    bool intersects(const AbstractState &other, int var) const;

    /*
      Separate the "wanted" values from the other values in the abstract domain
      and return the resulting two new Cartesian sets.
    */
    std::pair<CartesianSet, CartesianSet> split_domain(
        int var, const std::vector<int> &wanted) const;

    bool includes(const AbstractState &other) const;
    bool includes(const State &concrete_state) const;
    bool includes(const std::vector<FactPair> &facts) const;
    bool includes_any(int var, const std::unordered_set<int> &values) const;

    // IDs are consecutive, so they can be used to index states in vectors.
    int get_id() const;

    NodeID get_node_id() const;

    friend std::ostream &operator<<(std::ostream &os, const AbstractState &state) {
        return os << "#" << state.get_id() << state.cartesian_set;
    }

    // Create the initial, unrefined abstract state.
    static std::unique_ptr<AbstractState> get_trivial_abstract_state(
        const std::vector<int> &domain_sizes);
};
}

#endif
