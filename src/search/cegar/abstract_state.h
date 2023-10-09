#ifndef CEGAR_ABSTRACT_STATE_H
#define CEGAR_ABSTRACT_STATE_H

#include "cartesian_set.h"
#include "types.h"

#include <vector>

class ConditionsProxy;
struct FactPair;
class OperatorProxy;
class State;
class TaskProxy;

namespace cegar {
class Node;

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
    // Constructor for temporal abstract states used during flaw search.
    AbstractState(const std::vector<int> &domain_sizes, std::vector<FactPair> facts);

    bool domain_subsets_intersect(const AbstractState &other, int var) const;

    // Return if the variable is fully abstracted (all possible values)
    bool is_fully_abstracted(int var) const;

    // Return the size of var's abstract domain for this state.
    int count(int var) const;

    // Return a vector with the number of concrete states represented by each
    // variable.
    std::vector<int> count() const;

    bool contains(int var, int value) const;

    bool is_applicable(const OperatorProxy &op) const;
    bool is_backward_applicable(const OperatorProxy &op) const;
    std::vector<int> vars_not_backward_applicable(const OperatorProxy &op) const;

    // Return the Cartesian set in which applying "op" can lead to this state.
    CartesianSet regress(const OperatorProxy &op) const;

    /*
      Separate the "wanted" values from the other values in the abstract domain
      and return the resulting two new Cartesian sets.
    */
    std::pair<CartesianSet, CartesianSet> split_domain(
        int var, const std::vector<int> &wanted) const;

    bool intersects(const AbstractState &other) const;
    bool includes(const AbstractState &other) const;
    bool includes(const State &concrete_state) const;
    bool includes(const FactPair &fact) const;
    bool includes(const std::vector<FactPair> &facts) const;

    // IDs are consecutive, so they can be used to index states in vectors.
    int get_id() const;

    NodeID get_node_id() const;

    CartesianSet get_cartesian_set() const;
    AbstractState intersection(const AbstractState &other) const;

    friend std::ostream &operator<<(std::ostream &os, const AbstractState &state) {
        return os << "#" << state.get_id() << state.cartesian_set;
    }

    // Create the initial, unrefined abstract state.
    static std::unique_ptr<AbstractState> get_trivial_abstract_state(
        const std::vector<int> &domain_sizes);
};
}

#endif
