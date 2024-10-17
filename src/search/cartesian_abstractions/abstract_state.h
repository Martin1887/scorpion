#ifndef CARTESIAN_ABSTRACTIONS_ABSTRACT_STATE_H
#define CARTESIAN_ABSTRACTIONS_ABSTRACT_STATE_H

#include "types.h"

#include "../task_utils/cartesian_set.h"
#include "../task_utils/cartesian_state.h"

#include <vector>

class ConditionsProxy;
struct FactPair;
class OperatorProxy;
class State;
class TaskProxy;

using namespace cartesian_set;
using namespace cartesian_state;

namespace cartesian_abstractions {
/*
  Store the Cartesian set and the ID of the node in the refinement hierarchy
  for an abstract state.
*/
class AbstractState : public CartesianState {
    int state_id;

    // This state's node in the refinement hierarchy.
    NodeID node_id;

public:
    AbstractState(int state_id, NodeID node_id, CartesianSet &&cartesian_set);

    // IDs are consecutive, so they can be used to index states in vectors.
    int get_id() const;

    NodeID get_node_id() const;

    void set_node_id(NodeID new_node_id);

    friend std::ostream &operator<<(std::ostream &os, const AbstractState &state) {
        return os << "#" << state.get_id() << state.cartesian_set;
    }

    // Create the initial, unrefined abstract state.
    static std::unique_ptr<AbstractState> get_trivial_abstract_state(
        const std::vector<int> &domain_sizes);
};
}

#endif
