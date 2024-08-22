#include "cartesian_state.h"

#include "disambiguated_operator.h"

#include "../utils/memory.h"

#include <cassert>
#include <unordered_set>

using namespace cartesian_set;
using namespace disambiguation;
using namespace std;

namespace cartesian_state {
CartesianState::CartesianState(CartesianSet &&cartesian_set)
    : cartesian_set(move(cartesian_set)) {
}
CartesianState::CartesianState(
    const vector<int> &domain_sizes, vector<FactPair> facts)
    : cartesian_set(domain_sizes, facts) {
}

bool CartesianState::is_fully_abstracted(int var) const {
    return cartesian_set.all_values_set(var);
}

int CartesianState::count(int var) const {
    return cartesian_set.count(var);
}

vector<int> CartesianState::count() const {
    vector<int> states;
    int n_vars = cartesian_set.n_vars();
    for (int var = 0; var < n_vars; var++) {
        states.push_back(cartesian_set.count(var));
    }

    return states;
}

bool CartesianState::is_spurious() const {
    return cartesian_set.is_empty();
}

bool CartesianState::contains(int var, int value) const {
    return cartesian_set.test(var, value);
}

pair<CartesianSet, CartesianSet> CartesianState::split_domain(
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

bool CartesianState::is_applicable(const OperatorProxy &op) const {
    for (FactProxy precondition : op.get_preconditions()) {
        if (!includes(precondition.get_pair()))
            return false;
    }
    return true;
}
bool CartesianState::is_applicable(const DisambiguatedOperator &op) const {
    return intersects(op.get_precondition());
}
bool CartesianState::is_backward_applicable(const OperatorProxy &op) const {
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
bool CartesianState::is_backward_applicable(const DisambiguatedOperator &op) const {
    for (FactPair fact : op.get_effect()) {
        if (!includes(fact)) {
            return false;
        }
    }
    CartesianSet preconds = op.get_precondition().get_cartesian_set();
    for (int var = 0; var < preconds.n_vars(); var++) {
        if (!op.has_effect(var) && !cartesian_set.intersects(preconds, var)) {
            return false;
        }
    }

    return true;
}
vector<int> CartesianState::vars_not_backward_applicable(const OperatorProxy &op) const {
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
vector<int> CartesianState::vars_not_backward_applicable(const DisambiguatedOperator &op) const {
    vector<int> not_applicable{};
    for (FactPair fact : op.get_effect()) {
        if (!includes(fact)) {
            // Effects can have at most 1 value at each var.
            not_applicable.push_back(fact.var);
        }
    }
    CartesianSet preconds = op.get_precondition().get_cartesian_set();
    for (int var = 0; var < preconds.n_vars(); var++) {
        if (!op.has_effect(var) && !cartesian_set.intersects(preconds, var)) {
            // Vars are pushed at most once in this loop and without effects
            // they cannot be already in the not applicable vector.
            not_applicable.push_back(var);
        }
    }

    return not_applicable;
}

CartesianSet CartesianState::regress(const OperatorProxy &op) const {
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
CartesianSet CartesianState::regress(const DisambiguatedOperator &op) const {
    CartesianSet regression(cartesian_set);
    // If the operator has no effects in the variable, then the intersection
    // with preconds is set except that intersection is empty
    // (inapplicable operator), setting  preconditions otherwise.
    // Removing effects is not needed because preconditions have
    // value for all variables.
    CartesianSet preconds = op.get_precondition().get_cartesian_set();
    for (int var = 0; var < preconds.n_vars(); var++) {
        if (op.has_effect(var) || !regression.intersects(preconds, var)) {
            // TODO: This should be able to be done faster.
            regression.set_values(var, preconds.get_values(var));
        } else if (!op.has_effect(var)) {
            regression.set_values(var, cartesian_set.var_intersection(preconds, var));
        }
    }
    return regression;
}

CartesianSet CartesianState::progress(const OperatorProxy &op) const {
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
CartesianSet CartesianState::progress(const DisambiguatedOperator &op) const {
    CartesianSet progression(cartesian_set);
    // Not met preconditions are also set because the operator could be inapplicable.
    CartesianSet preconds = op.get_precondition().get_cartesian_set();
    for (int var = 0; var < preconds.n_vars(); var++) {
        if (!progression.intersects(preconds, var)) {
            // TODO: This should be able to be done faster.
            progression.set_values(var, preconds.get_values(var));
        }
    }
    for (FactPair effect : op.get_effect()) {
        progression.set_single_value(effect.var, effect.value);
    }
    return progression;
}

CartesianSet CartesianState::undeviate(const CartesianState &mapped) const {
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

bool CartesianState::domain_subsets_intersect(const CartesianState &other, int var) const {
    return cartesian_set.intersects(other.cartesian_set, var);
}

bool CartesianState::includes(const State &concrete_state) const {
    for (FactProxy fact : concrete_state) {
        if (!cartesian_set.test(fact.get_variable().get_id(), fact.get_value()))
            return false;
    }
    return true;
}

bool CartesianState::includes(const FactPair &fact) const {
    return cartesian_set.test(fact.var, fact.value);
}

bool CartesianState::includes(const vector<FactPair> &facts) const {
    for (const FactPair &fact : facts) {
        if (!includes(fact))
            return false;
    }
    return true;
}

bool CartesianState::includes(const CartesianState &other) const {
    return cartesian_set.is_superset_of(other.cartesian_set);
}

bool CartesianState::intersects(const CartesianState &other) const {
    return cartesian_set.intersects(other.cartesian_set);
}
CartesianSet CartesianState::get_cartesian_set() const {
    return cartesian_set;
}

CartesianState CartesianState::intersection(const CartesianState &other) const {
    return CartesianState(cartesian_set.intersection(other.get_cartesian_set()));
}

unique_ptr<CartesianState> CartesianState::get_trivial_abstract_state(
    const vector<int> &domain_sizes) {
    return utils::make_unique_ptr<CartesianState>(CartesianSet(domain_sizes));
}
}
