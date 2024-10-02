#include "cartesian_state.h"

#include "disambiguated_operator.h"

#include "../task_utils/cartesian_set_facts_proxy_iterator.h"
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
    int n_vars = cartesian_set.get_n_vars();
    for (int var = 0; var < n_vars; var++) {
        states.push_back(cartesian_set.count(var));
    }

    return states;
}

bool CartesianState::is_spurious() const {
    return cartesian_set.is_empty();
}

bool CartesianState::got_empty() {
    return cartesian_set.got_empty();
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
    return !is_spurious() && intersects(op.get_precondition());
}
bool CartesianState::is_applicable(const DisambiguatedOperator &op, const vector<int> &vars) const {
    if (is_spurious()) {
        return false;
    } else {
        for (int var : vars) {
            if (!is_applicable(op, var)) {
                return false;
            }
        }
    }

    return true;
}
bool CartesianState::is_applicable(const DisambiguatedOperator &op, int var) const {
    return intersects(op.get_precondition(), var);
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
    if (is_spurious()) {
        return false;
    } else {
        int n_vars = cartesian_set.get_n_vars();
        for (int var = 0; var < n_vars; var++) {
            if (!is_backward_applicable(op, var)) {
                return false;
            }
        }
        return true;
    }
}
bool CartesianState::is_backward_applicable(const DisambiguatedOperator &op, int var) const {
    if (is_spurious()) {
        return false;
    } else {
        int eff_value = op.get_effect(var);
        if (eff_value != -1) {
            return cartesian_set.test(var, eff_value);
        } else {
            return cartesian_set.intersects(op.get_post().get_cartesian_set(), var);
        }
    }
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
    int n_vars = cartesian_set.get_n_vars();
    not_applicable.reserve(n_vars);
    for (int var = 0; var < n_vars; var++) {
        if (!is_backward_applicable(op, var)) {
            not_applicable.push_back(var);
        }
    }

    return not_applicable;
}

bool CartesianState::reach_with_op(const CartesianState &other, const disambiguation::DisambiguatedOperator &op) const {
    if (other.is_spurious()) {
        return false;
    }
    const CartesianSet &other_set = other.get_cartesian_set();
    const CartesianSet &pre = op.get_precondition().get_cartesian_set();
    int n_vars = cartesian_set.get_n_vars();
    for (int var = 0; var < n_vars; var++) {
        if (!reach_with_op(other_set, pre, op.get_effect(var), var)) {
            return false;
        }
    }

    return true;
}

bool CartesianState::reach_with_op(const CartesianState &other, const disambiguation::DisambiguatedOperator &op, const vector<int> &vars) const {
    if (other.is_spurious()) {
        return false;
    }
    const CartesianSet &other_set = other.get_cartesian_set();
    const CartesianSet &pre = op.get_precondition().get_cartesian_set();
    for (int var : vars) {
        if (!reach_with_op(other_set, pre, op.get_effect(var), var)) {
            return false;
        }
    }

    return true;
}

bool CartesianState::reach_with_op(const CartesianSet &other_set, const CartesianSet &pre, int var_effect, int var) const {
    if (var_effect != -1) {
        return other_set.test(var, var_effect);
    } else {
        return other_set.intersects_intersection(cartesian_set, pre, var);
    }
}

bool CartesianState::reach_with_inapplicable_op(const CartesianState &other, const disambiguation::DisambiguatedOperator &op) const {
    if (other.is_spurious()) {
        return false;
    } else {
        const CartesianSet &other_set = other.get_cartesian_set();
        const CartesianSet &pre = op.get_precondition().get_cartesian_set();
        const CartesianSet &post = op.get_post().get_cartesian_set();
        int n_vars = cartesian_set.get_n_vars();
        for (int var = 0; var < n_vars; var++) {
            if (!reach_with_inapplicable_op(other_set, pre, post, op.get_effect(var), var)) {
                return false;
            }
        }

        return true;
    }
}
bool CartesianState::reach_with_inapplicable_op(const CartesianSet &other_set, const CartesianSet &pre, const CartesianSet &post, int var_effect, int var) const {
    if (var_effect != -1) {
        return other_set.test(var, var_effect);
    } else if (!pre.intersects(other_set, var)) {
        return post.intersects(other_set, var);
    } else {
        return other_set.intersects_intersection(cartesian_set, pre, var);
    }
}

bool CartesianState::reach_backwards_with_op(const CartesianState &other, const disambiguation::DisambiguatedOperator &op) const {
    if (other.is_spurious()) {
        return false;
    }
    const CartesianSet &pre = op.get_precondition().get_cartesian_set();
    const CartesianSet &other_set = other.get_cartesian_set();
    int n_vars = cartesian_set.get_n_vars();
    for (int var = 0; var < n_vars; var++) {
        if (op.has_effect(var)) {
            if (!pre.intersects(other_set, var)) {
                return false;
            }
        } else if (!cartesian_set.intersects(other_set, var)) {
            return false;
        }
    }

    return true;
}
bool CartesianState::reach_backwards_with_inapplicable_op(const CartesianState &other, const disambiguation::DisambiguatedOperator &op) const {
    if (other.is_spurious()) {
        return false;
    }
    const CartesianSet &pre = op.get_precondition().get_cartesian_set();
    const CartesianSet &other_set = other.get_cartesian_set();
    int n_vars = cartesian_set.get_n_vars();
    for (int var = 0; var < n_vars; var++) {
        if (op.has_effect(var) || !cartesian_set.intersects(pre, var)) {
            if (!pre.intersects(other_set, var)) {
                return false;
            }
        } else if (!cartesian_set.intersects(other_set, var)) {
            return false;
        }
    }

    return true;
}

void CartesianState::regress(const OperatorProxy &op) {
    for (EffectProxy effect : op.get_effects()) {
        int var_id = effect.get_fact().get_variable().get_id();
        cartesian_set.add_all(var_id);
    }
    for (FactProxy precondition : op.get_preconditions()) {
        int var_id = precondition.get_variable().get_id();
        cartesian_set.set_single_value(var_id, precondition.get_value());
    }
}
void CartesianState::regress(const DisambiguatedOperator &op) {
    // If the operator has no effects in the variable, then the intersection
    // with preconds is set except that intersection is empty
    // (inapplicable operator), setting  preconditions otherwise.
    // Removing effects is not needed because preconditions have
    // value for all variables.
    const CartesianSet &preconds = op.get_precondition().get_cartesian_set();
    int n_vars = cartesian_set.get_n_vars();
    for (int var = 0; var < n_vars; var++) {
        if (op.has_effect(var) || !cartesian_set.intersects(preconds, var)) {
            cartesian_set.set_values(var, preconds);
        } else {
            cartesian_set.set_intersection_values(var, preconds);
        }
    }
}

void CartesianState::progress(const OperatorProxy &op) {
    // Preconditions are also set because the operator could be not applicable.
    for (FactProxy precondition : op.get_preconditions()) {
        int var_id = precondition.get_variable().get_id();
        cartesian_set.set_single_value(var_id, precondition.get_value());
    }
    for (EffectProxy effect : op.get_effects()) {
        int var_id = effect.get_fact().get_variable().get_id();
        cartesian_set.set_single_value(var_id, effect.get_fact().get_value());
    }
}
void CartesianState::progress(const DisambiguatedOperator &op) {
    const CartesianSet &pre = op.get_precondition().get_cartesian_set();
    const CartesianSet &post = op.get_post().get_cartesian_set();
    int n_vars = cartesian_set.get_n_vars();
    for (int var = 0; var < n_vars; var++) {
        int eff_value = op.get_effect(var);
        if (eff_value != -1) {
            cartesian_set.set_single_value(var, eff_value);
        } else if (!cartesian_set.intersects(pre, var)) {
            cartesian_set.set_values(var, post);
        }
    }
}

void CartesianState::undeviate(const CartesianState &mapped) {
    int n_vars = cartesian_set.get_n_vars();
    for (int var = 0; var < n_vars; var++) {
        if (!domain_subsets_intersect(mapped, var)) {
            cartesian_set.remove_all(var);
            for (auto &&[fact_var, fact_value] : mapped.get_cartesian_set().iter(var)) {
                cartesian_set.add(var, fact_value);
            }
        }
    }
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

bool CartesianState::includes(int var, int value) const {
    return cartesian_set.test(var, value);
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
bool CartesianState::intersects(const CartesianState &other, int var) const {
    return cartesian_set.intersects(other.cartesian_set, var);
}
const CartesianSet &CartesianState::get_cartesian_set() const {
    return cartesian_set;
}
CartesianSet CartesianState::clone_cartesian_set() const {
    return cartesian_set;
}
void CartesianState::set_cartesian_set(CartesianSet &&other) {
    cartesian_set = move(other);
}

CartesianState CartesianState::intersection(const CartesianState &other) const {
    return CartesianState(cartesian_set.intersection(other.get_cartesian_set()));
}

unique_ptr<CartesianState> CartesianState::get_trivial_abstract_state(
    const vector<int> &domain_sizes) {
    return utils::make_unique_ptr<CartesianState>(CartesianSet(domain_sizes));
}
}
