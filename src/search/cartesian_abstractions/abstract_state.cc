#include "abstract_state.h"

#include "../utils/memory.h"

#include <cassert>

using namespace std;

namespace cartesian_abstractions {
AbstractState::AbstractState(
    int state_id, NodeID node_id, CartesianSet &&cartesian_set)
    : CartesianState(move(cartesian_set)),
      state_id(state_id),
      node_id(node_id) {
}

int AbstractState::get_id() const {
    return state_id;
}

NodeID AbstractState::get_node_id() const {
    return node_id;
}

unique_ptr<AbstractState> AbstractState::get_trivial_abstract_state(
    const vector<int> &domain_sizes) {
    return utils::make_unique_ptr<AbstractState>(0, 0, CartesianSet(domain_sizes));
}
}
