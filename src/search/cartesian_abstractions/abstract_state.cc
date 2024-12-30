#include "abstract_state.h"

#include "refinement_hierarchy.h"
#include "types.h"
#include "utils.h"

#include "../utils/memory.h"

#include <algorithm>
#include <cassert>
#include <unordered_set>

using namespace std;

namespace cartesian_abstractions {
AbstractState::AbstractState(
    int state_id, NodeID node_id, CartesianSet &&cartesian_set)
    : state_id(state_id),
      node_id(node_id),
      cartesian_set(move(cartesian_set)) {
}

AbstractState::AbstractState(
    int state_id, NodeID node_id, const vector<int> &domain_sizes, vector<FactPair> facts)
    : state_id(state_id),
      node_id(node_id),
      cartesian_set(domain_sizes, facts) {
}

int AbstractState::n_vars() const {
    return cartesian_set.n_vars();
}

const CartesianSet &AbstractState::get_cartesian_set() const {
    return cartesian_set;
}
CartesianSet AbstractState::clone_cartesian_set() const {
    return cartesian_set;
}

int AbstractState::count(int var) const {
    return cartesian_set.count(var);
}

bool AbstractState::contains(int var, int value) const {
    return cartesian_set.test(var, value);
}

pair<CartesianSet, CartesianSet> AbstractState::split_domain(
    int var, const vector<int> &wanted) const {
    int num_wanted = wanted.size();
    utils::unused_variable(num_wanted);
    // We can only refine for variables with at least two values.
    assert(num_wanted >= 1);
    assert(cartesian_set.count(var) > num_wanted);

    CartesianSet v1_cartesian_set(cartesian_set);
    // cartesian_set is not used anymore.
    CartesianSet v2_cartesian_set(move(cartesian_set));

    v2_cartesian_set.remove_all(var);
    for (int value : wanted) {
        // The wanted value has to be in the set of possible values.
        assert(v1_cartesian_set.test(var, value));

        // In v1 var can have all of the previous values except the wanted ones.
        v1_cartesian_set.remove(var, value);

        // In v2 var can only have the wanted values.
        v2_cartesian_set.add(var, value);
    }
    assert(v2_cartesian_set.count(var) == num_wanted);
    return make_pair(move(v1_cartesian_set), move(v2_cartesian_set));
}

bool AbstractState::is_backward_applicable(const vector<unordered_set<int>> &post) const {
    int n_vars = cartesian_set.n_vars();
    for (int var = 0; var < n_vars; var++) {
        if (!is_backward_applicable(var, post[var])) {
            return false;
        }
    }
    return true;
}
bool AbstractState::is_backward_applicable(int var, const unordered_set<int> &var_post) const {
    return var_post.empty() || var_post.contains(UNDEFINED) || includes_any(var, var_post);
}

bool AbstractState::reach_backwards_with_op(const AbstractState &other, const OperatorProxy &op) const {
    int n_vars = cartesian_set.n_vars();
    vector<bool> fixed_value_vars(n_vars, false);
    const CartesianSet &other_set = other.get_cartesian_set();
    // Variables with precondition must have precondition value,
    // variables on always fired effects can have any value,
    // variables on conditional effects can have any value if conditions are
    // satisfied, the rest of variables must intersect with this state.
    for (const FactProxy &pre : op.get_preconditions()) {
        int var = pre.get_variable().get_id();
        if (!other_set.test(var, pre.get_value())) {
            return false;
        }
        fixed_value_vars[var] = true;
    }
    for (const EffectProxy &eff : op.get_effects()) {
        int var = eff.get_fact().get_variable().get_id();
        if (!fixed_value_vars[var] && cartesian_set.test(var, eff.get_fact().get_value())) {
            bool conds_satisifed = true;
            for (const FactProxy &cond : eff.get_conditions()) {
                if (!other_set.test(cond.get_variable().get_id(), cond.get_value())) {
                    conds_satisifed = false;
                    break;
                }
            }
            if (conds_satisifed) {
                // Any value is good.
                fixed_value_vars[var] = true;
            }
        }
    }
    for (int var = 0; var < n_vars; var++) {
        if (!fixed_value_vars[var] && !cartesian_set.intersects(other_set, var)) {
            return false;
        }
    }

    return true;
}

void AbstractState::regress(const OperatorProxy &op) {
    for (EffectProxy effect : op.get_effects()) {
        int var_id = effect.get_fact().get_variable().get_id();
        // For conditional effects, or the predecessor has this value or the
        // conditions of the effect are satisfied. This is not Cartesian,
        // but we overapproximate it to the Cartesian set that satisfies both.
        // Since we don't know if the effect result is due to satisfied
        // conditions we have to set all values in the variable anyway.
        if (effect.get_conditions().empty()) {
            assert(cartesian_set.test(var_id, effect.get_fact().get_value()));
            cartesian_set.add_all(var_id);
        } else if (cartesian_set.test(var_id, effect.get_fact().get_value())) {
            // Only effects true in this state are taken into account
            // (not fired effects must not be taken into account).
            for (const FactProxy &cond : effect.get_conditions()) {
                cartesian_set.add(cond.get_variable().get_id(), cond.get_value());
            }
            cartesian_set.add_all(var_id);
        }
    }
    for (FactProxy precondition : op.get_preconditions()) {
        int var_id = precondition.get_variable().get_id();
        cartesian_set.set_single_value(var_id, precondition.get_value());
    }
}

bool AbstractState::domain_subsets_intersect(const CartesianSet &other, const vector<int> &vars) const {
    for (int var : vars) {
        if (!domain_subsets_intersect(other, var)) {
            return false;
        }
    }

    return true;
}

bool AbstractState::domain_subsets_intersect(const CartesianSet &other, const vector<bool> &vars) const {
    int i = 0;
    for (bool var : vars) {
        if (var && !domain_subsets_intersect(other, i)) {
            return false;
        }
        i++;
    }

    return true;
}

bool AbstractState::domain_subsets_intersect(const CartesianSet &other, int var) const {
    return cartesian_set.intersects(other, var);
}

bool AbstractState::domain_subsets_intersect(const AbstractState &other, int var) const {
    return cartesian_set.intersects(other.cartesian_set, var);
}

bool AbstractState::includes(const AbstractState &other) const {
    return cartesian_set.is_superset_of(other.cartesian_set);
}

bool AbstractState::includes(const State &concrete_state) const {
    for (FactProxy fact : concrete_state) {
        if (!cartesian_set.test(fact.get_variable().get_id(), fact.get_value()))
            return false;
    }
    return true;
}

bool AbstractState::includes(const vector<FactPair> &facts) const {
    for (const FactPair &fact : facts) {
        if (!cartesian_set.test(fact.var, fact.value))
            return false;
    }
    return true;
}

bool AbstractState::includes_any(int var, const std::unordered_set<int> &values) const {
    for (int value : values) {
        if (contains(var, value)) {
            return true;
        }
    }

    return false;
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
