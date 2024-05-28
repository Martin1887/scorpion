#ifndef CARTESIAN_ABSTRACTIONS_REFINEMENT_HIERARCHY_H
#define CARTESIAN_ABSTRACTIONS_REFINEMENT_HIERARCHY_H

#include "abstraction.h"
#include "shortest_paths.h"
#include "types.h"

#include <cassert>
#include <memory>
#include <ostream>
#include <utility>
#include <vector>

class AbstractTask;
class State;

namespace cartesian_abstractions {
class Node {
    /*
      While right_child is always the node of a (possibly split)
      abstract state, left_child may be a helper node. We add helper
      nodes to the hierarchy to allow for efficient lookup in case more
      than one fact is split off a state.
    */
    NodeID left_child;
    NodeID right_child;

    // This is the split variable for inner nodes and UNDEFINED for leaf nodes.
    int var;

    // This is the split value for inner nodes and the state ID for leaf nodes.
    int value;

    bool information_is_valid() const;

public:
    explicit Node(int state_id);

    bool is_split() const;

    void split(int var, int value, NodeID left_child, NodeID right_child);

    int get_var() const {
        assert(is_split());
        return var;
    }

    NodeID get_child(int val) const {
        assert(is_split());
        if (val == value)
            return right_child;
        return left_child;
    }

    NodeID get_left_child() const {
        return left_child;
    }

    NodeID get_right_child() const {
        return right_child;
    }

    int get_state_id() const {
        assert(!is_split());
        return value;
    }

    friend std::ostream &operator<<(std::ostream &os, const Node &node);
};

/*
  This class stores the refinement hierarchy of a Cartesian
  abstraction. The hierarchy forms a DAG with inner nodes for each
  split and leaf nodes for the abstract states.

  It is used for efficient lookup of abstract states during search.

  Inner nodes correspond to abstract states that have been split (or
  helper nodes, see below). Leaf nodes correspond to the current
  (unsplit) states in an abstraction. The use of helper nodes makes
  this structure a directed acyclic graph (instead of a tree).
*/
class RefinementHierarchy {
    std::shared_ptr<AbstractTask> task;
    std::vector<Node> nodes;

    NodeID add_node(int state_id);
    NodeID get_node_id(const State &state) const;

    // This class is only used to retrieve the number of useless nodes.
    class LeftChildNode {
public:
        NodeID left_node_id;
        NodeID sibling_id;
        int dist;
        int sibling_dist;
        std::shared_ptr<LeftChildNode> parent;
        bool is_child_of_right_node;

        LeftChildNode(NodeID left_node_id,
                      NodeID sibling_id,
                      int dist,
                      int sibling_dist,
                      std::shared_ptr<LeftChildNode> parent,
                      bool is_child_of_right_node = false)
            : left_node_id(left_node_id),
              sibling_id(sibling_id),
              dist(dist),
              sibling_dist(sibling_dist),
              parent(parent),
              is_child_of_right_node(is_child_of_right_node) {
        }
    };

    // Get the bottom leaf nodes (where both siblings are leaf) with their distance.
    std::vector<std::shared_ptr<LeftChildNode>> get_leaf_nodes(const std::vector<int> &goal_distances,
                                                               std::shared_ptr<LeftChildNode> struct_node = {}) const;

public:
    explicit RefinementHierarchy(const std::shared_ptr<AbstractTask> &task);

    /*
      Update the split tree for the new split. Additionally to the left
      and right child nodes add |values|-1 helper nodes that all have
      the right child as their right child and the next helper node as
      their left child.
    */
    std::pair<NodeID, NodeID> split(
        NodeID node_id, int var, const std::vector<int> &values,
        int left_state_id, int right_state_id);

    int get_abstract_state_id(const State &state) const;
    friend int Abstraction::get_abstract_state_id(const State &state) const;

    int get_num_nodes() const {
        return nodes.size();
    }

    int n_useless_refinements(const std::vector<int> &goal_distances) const;
};
}

#endif
