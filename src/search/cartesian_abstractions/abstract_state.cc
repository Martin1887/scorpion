#include "abstract_state.h"

#include "refinement_hierarchy.h"
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
    const vector<int> &domain_sizes, vector<FactPair> facts)
    : state_id(-1),
      node_id(-1),
      cartesian_set(domain_sizes, facts) {
}

bool AbstractState::is_fully_abstracted(int var) const {
    return cartesian_set.all_values_set(var);
}

int AbstractState::count(int var) const {
    return cartesian_set.count(var);
}

vector<int> AbstractState::count() const {
    vector<int> states;
    int n_vars = cartesian_set.n_vars();
    for (int var = 0; var < n_vars; var++) {
        states.push_back(cartesian_set.count(var));
    }

    return states;
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
    CartesianSet v2_cartesian_set(cartesian_set);

    v2_cartesian_set.remove_all(var);
    for (int value : wanted) {
        // The wanted value has to be in the set of possible values.
        assert(cartesian_set.test(var, value));

        // In v1 var can have all of the previous values except the wanted ones.
        v1_cartesian_set.remove(var, value);

        // In v2 var can only have the wanted values.
        v2_cartesian_set.add(var, value);
    }
    assert(v1_cartesian_set.count(var) == cartesian_set.count(var) - num_wanted);
    assert(v2_cartesian_set.count(var) == num_wanted);
    return make_pair(v1_cartesian_set, v2_cartesian_set);
}

bool AbstractState::is_applicable(const OperatorProxy &op) const {
    for (FactProxy precondition : op.get_preconditions()) {
        if (!includes(precondition.get_pair()))
            return false;
    }
    return true;
}
bool AbstractState::is_backward_applicable(const OperatorProxy &op) const {
    unordered_set<int> effect_vars{};
    for (EffectProxy ef : op.get_effects()) {
        effect_vars.insert(ef.get_fact().get_variable().get_id());
        if (!includes(ef.get_fact().get_pair())) {
            return false;
        }
    }
    for (FactProxy cond : op.get_preconditions()) {
        // Prevail conditions (variable is in the preconditions but not in the
        // effects) must also be checked because though its value does not
        // change in the target state it must be included.
        if (effect_vars.count(cond.get_variable().get_id()) == 0) {
            if (!includes(cond.get_pair())) {
                return false;
            }
        }
    }

    return true;
}
vector<int> AbstractState::vars_not_backward_applicable(const OperatorProxy &op) const {
    vector<int> not_applicable{};
    unordered_set<int> effect_vars{};
    for (EffectProxy ef : op.get_effects()) {
        effect_vars.insert(ef.get_fact().get_variable().get_id());
        if (!includes(ef.get_fact().get_pair())) {
            not_applicable.push_back(ef.get_fact().get_variable().get_id());
        }
    }
    for (FactProxy cond : op.get_preconditions()) {
        // Prevail conditions (variable is in the preconditions but not in the
        // effects) must also be checked because though its value does not
        // change in the target state it must be included.
        if (effect_vars.count(cond.get_variable().get_id()) == 0) {
            if (!includes(cond.get_pair())) {
                not_applicable.push_back(cond.get_variable().get_id());
            }
        }
    }

    return not_applicable;
}

CartesianSet AbstractState::regress(const OperatorProxy &op) const {
    CartesianSet regression(cartesian_set);
    for (EffectProxy effect : op.get_effects()) {
        int var_id = effect.get_fact().get_variable().get_id();
        regression.add_all(var_id);
    }
    for (FactProxy precondition : op.get_preconditions()) {
        int var_id = precondition.get_variable().get_id();
        regression.set_single_value(var_id, precondition.get_value());
    }
    return regression;
}

CartesianSet AbstractState::progress(const OperatorProxy &op) const {
    CartesianSet progression(cartesian_set);
    // Preconditions are also set because the operator could be not applicable.
    for (FactProxy precondition : op.get_preconditions()) {
        int var_id = precondition.get_variable().get_id();
        progression.set_single_value(var_id, precondition.get_value());
    }
    for (EffectProxy effect : op.get_effects()) {
        int var_id = effect.get_fact().get_variable().get_id();
        progression.set_single_value(var_id, effect.get_fact().get_value());
    }
    return progression;
}

CartesianSet AbstractState::undeviate(const AbstractState &mapped) const {
    CartesianSet undeviated(cartesian_set);

    for (int var = 0; var < cartesian_set.n_vars(); var++) {
        if (!domain_subsets_intersect(mapped, var)) {
            undeviated.remove_all(var);
            for (int value : mapped.get_cartesian_set().get_values(var)) {
                undeviated.add(var, value);
            }
        }
    }

    return undeviated;
}

bool AbstractState::domain_subsets_intersect(const AbstractState &other, int var) const {
    return cartesian_set.intersects(other.cartesian_set, var);
}

bool AbstractState::includes(const State &concrete_state) const {
    for (FactProxy fact : concrete_state) {
        if (!cartesian_set.test(fact.get_variable().get_id(), fact.get_value()))
            return false;
    }
    return true;
}

bool AbstractState::includes(const FactPair &fact) const {
    return cartesian_set.test(fact.var, fact.value);
}

bool AbstractState::includes(const vector<FactPair> &facts) const {
    for (const FactPair &fact : facts) {
        if (!includes(fact))
            return false;
    }
    return true;
}

bool AbstractState::includes(const AbstractState &other) const {
    return cartesian_set.is_superset_of(other.cartesian_set);
}

bool AbstractState::intersects(const AbstractState &other) const {
    return cartesian_set.intersects(other.cartesian_set);
}

int AbstractState::get_id() const {
    return state_id;
}

NodeID AbstractState::get_node_id() const {
    return node_id;
}

CartesianSet AbstractState::get_cartesian_set() const {
    return cartesian_set;
}

AbstractState AbstractState::intersection(const AbstractState &other) const {
    return AbstractState(state_id, node_id, cartesian_set.intersection(other.get_cartesian_set()));
}

unique_ptr<AbstractState> AbstractState::get_trivial_abstract_state(
    const vector<int> &domain_sizes) {
    return utils::make_unique_ptr<AbstractState>(0, 0, CartesianSet(domain_sizes));
}
}
