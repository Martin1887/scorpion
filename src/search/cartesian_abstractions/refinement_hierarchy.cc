#include "refinement_hierarchy.h"

#include "../task_proxy.h"

using namespace std;

namespace cartesian_abstractions {
Node::Node(int state_id)
    : left_child(UNDEFINED),
      right_child(UNDEFINED),
      var(UNDEFINED),
      value(state_id) {
    assert(!is_split());
}

bool Node::information_is_valid() const {
    return value != UNDEFINED && (
        // leaf node
        (left_child == UNDEFINED && right_child == UNDEFINED && var == UNDEFINED) ||
        // inner node
        (left_child != UNDEFINED && right_child != UNDEFINED && var != UNDEFINED));
}

bool Node::is_split() const {
    assert(information_is_valid());
    return left_child != UNDEFINED;
}

void Node::split(int var, int value, NodeID left_child, NodeID right_child) {
    this->var = var;
    this->value = value;
    this->left_child = left_child;
    this->right_child = right_child;
    assert(is_split());
}



ostream &operator<<(ostream &os, const Node &node) {
    if (node.is_split()) {
        return os << "<Inner Node: var=" << node.var << " value=" << node.value
                  << " left=" << node.left_child << " right=" << node.right_child << ">";
    } else {
        return os << "<Leaf Node: state=" << node.value << ">";
    }
}


RefinementHierarchy::RefinementHierarchy(const shared_ptr<AbstractTask> &task)
    : task(task) {
    nodes.emplace_back(0);
}

NodeID RefinementHierarchy::add_node(int state_id) {
    NodeID node_id = nodes.size();
    nodes.emplace_back(state_id);
    return node_id;
}

NodeID RefinementHierarchy::get_node_id(const State &state) const {
    NodeID id = 0;
    while (nodes[id].is_split()) {
        id = nodes[id].get_child(state[nodes[id].get_var()].get_value());
    }
    return id;
}

pair<NodeID, NodeID> RefinementHierarchy::split(
    NodeID node_id, int var, const vector<int> &values, int left_state_id, int right_state_id) {
    NodeID helper_id = node_id;
    NodeID right_child_id = add_node(right_state_id);
    for (int value : values) {
        NodeID new_helper_id = add_node(left_state_id);
        nodes[helper_id].split(var, value, new_helper_id, right_child_id);
        helper_id = new_helper_id;
    }
    return make_pair(helper_id, right_child_id);
}

int RefinementHierarchy::get_abstract_state_id(const State &state) const {
    TaskProxy subtask_proxy(*task);
    if (subtask_proxy.needs_to_convert_ancestor_state(state)) {
        State subtask_state = subtask_proxy.convert_ancestor_state(state);
        return nodes[get_node_id(subtask_state)].get_state_id();
    } else {
        return nodes[get_node_id(state)].get_state_id();
    }
}

vector<shared_ptr<RefinementHierarchy::LeftChildNode>> RefinementHierarchy::get_leaf_nodes(const vector<int> &goal_distances,
                                                                                           shared_ptr<LeftChildNode> struct_node) const {
    if (!struct_node) {
        // The root node.
        struct_node = make_shared<LeftChildNode>(0, -1, -1, -1, nullptr);
    }
    shared_ptr<Node> left_node = make_shared<Node>(nodes[struct_node->left_node_id]);
    shared_ptr<Node> right_node = nullptr;
    if (struct_node->parent) {
        right_node = make_shared<Node>(nodes[struct_node->sibling_id]);
    }
    vector<shared_ptr<LeftChildNode>> current_leaves{};
    vector<shared_ptr<Node>> current_nodes = {left_node, right_node};
    bool is_right_node = false;
    for (shared_ptr<Node> current_node : current_nodes) {
        if (current_node && current_node->is_split()) {
            NodeID right_child = current_node->get_right_child();
            // Get the bottom of the helper nodes.
            NodeID bottom_left_child = current_node->get_left_child();
            while (nodes[bottom_left_child].is_split() &&
                   nodes[bottom_left_child].get_right_child() == right_child) {
                bottom_left_child = nodes[bottom_left_child].get_left_child();
            }

            shared_ptr<LeftChildNode> child_struct_node = make_shared<LeftChildNode>(bottom_left_child, right_child, -1, -1, struct_node, is_right_node);
            if (nodes[bottom_left_child].is_split() || nodes[right_child].is_split()) {
                vector<shared_ptr<RefinementHierarchy::LeftChildNode>> leaves =
                    get_leaf_nodes(goal_distances, child_struct_node);
                current_leaves.insert(current_leaves.end(), leaves.begin(),
                                      leaves.end());
            }
            if (!nodes[bottom_left_child].is_split()) {
                child_struct_node->dist = goal_distances[nodes[bottom_left_child].get_state_id()];
            }
            if (!nodes[right_child].is_split()) {
                child_struct_node->sibling_dist = goal_distances[nodes[right_child].get_state_id()];
            }
            // The 2 nodes are leaf.
            if (!nodes[bottom_left_child].is_split() && !nodes[right_child].is_split()) {
                current_leaves.push_back(child_struct_node);
            }
        }
        is_right_node = true;
    }
    return current_leaves;
}

int RefinementHierarchy::n_useless_refinements(const vector<int> &goal_distances) const {
    vector<shared_ptr<RefinementHierarchy::LeftChildNode>> open_list = get_leaf_nodes(goal_distances);

    int useless_ref = 0;
    // width-first search of siblings with the same distance to goal.
    while (!open_list.empty()) {
        shared_ptr<RefinementHierarchy::LeftChildNode> struct_node = open_list.front();
        if (struct_node->dist == struct_node->sibling_dist) {
            useless_ref++;
            if (struct_node->parent) {
                if (struct_node->is_child_of_right_node) {
                    struct_node->parent->sibling_dist = struct_node->dist;
                } else {
                    struct_node->parent->dist = struct_node->dist;
                }
                // Only add the node to the open list if it is not already in.
                if (find(open_list.begin(), open_list.end(), struct_node->parent) == open_list.end()) {
                    open_list.push_back(struct_node->parent);
                }
            }
        }
        open_list.erase(open_list.begin());
    }

    return useless_ref;
}
}
