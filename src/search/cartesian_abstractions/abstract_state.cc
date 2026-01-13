#include "abstract_state.h"

#include "abstraction.h"
#include "refinement_hierarchy.h"
#include "transition_system.h"
#include "types.h"
#include "utils.h"

#include <algorithm>
#include <cassert>
#include <unordered_set>

using namespace std;

namespace cartesian_abstractions {
AbstractState::AbstractState(int state_id, NodeID node_id, CartesianSet &&cartesian_set)
    : state_id(state_id),
      node_id(node_id),
      cartesian_set(cartesian_set) {
}

AbstractState::AbstractState(
    int state_id, NodeID node_id, const vector<int> &domain_sizes, const vector<FactPair> &facts, bool partial_state)
    : state_id(state_id),
      node_id(node_id),
      cartesian_set(domain_sizes, facts, partial_state) {
}

vector<bool> AbstractState::get_possibly_triggered_effect_in_variable(const OperatorProxy &op,
                                                                      const Abstraction &abs,
                                                                      const optional<reference_wrapper<const CartesianSet>> &target) const {
    int nvars = n_vars();
    vector<bool> possibly_triggered_in_var(nvars, false);

    // Auxiliary variables.
    vector<bool> triggered_for_sure_in_var(nvars, false);
    vector<bool> some_possible_effect_in_var(nvars, false);

    int op_id = op.get_id();
    const vector<FactPair> &uncond_effects = abs.get_unconditional_effects(op_id);
    for (const FactPair &eff : uncond_effects) {
        possibly_triggered_in_var[eff.var] = true;
        some_possible_effect_in_var[eff.var] = true;
        triggered_for_sure_in_var[eff.var] = true;
    }
    const vector<CondEffect> &cond_effects = abs.get_conditional_effects(op_id);
    for (const CondEffect &ef : cond_effects) {
        some_possible_effect_in_var[ef.effect.var] = true;
    }

    int n_effects = cond_effects.size();
    deque<int> possibly_triggered_effects_queue{};
    for (int i = 0; i < n_effects; i++) {
        const CondEffect &ef = cond_effects[i];
        if (cartesian_set.test(ef.effect.var, ef.effect.value)) {
            // If conditions are not satisfied in the target and no effect exists
            // in the variable it is not possible.
            if (target.has_value()) {
                bool conds_satisfied = true;
                for (const FactPair &cond : ef.conds) {
                    if (!some_possible_effect_in_var[cond.var] &&
                        !target.value().get().test(cond.var, cond.value)) {
                        conds_satisfied = false;
                        break;
                    }
                }
                if (conds_satisfied) {
                    possibly_triggered_effects_queue.push_back(i);
                }
            } else {
                possibly_triggered_effects_queue.push_back(i);
            }
        }
    }
    // Fix point is checked at the end of the loop.
    while (true) {
        deque<int> next_possibly_triggered_effects_queue{};
        size_t prev_queue_size = possibly_triggered_effects_queue.size();
        while (!possibly_triggered_effects_queue.empty()) {
            int eff_index = possibly_triggered_effects_queue.front();
            const CondEffect &ef = cond_effects[eff_index];
            possibly_triggered_effects_queue.pop_front();
            bool possibly_triggered = true;
            bool triggered_for_sure = false;
            for (const FactPair &cond : ef.conds) {
                if (ef.effect.var == cond.var ||
                    cartesian_set.test(cond.var, cond.value) ||
                    triggered_for_sure_in_var[cond.var]) {
                    triggered_for_sure = true;
                } else if (!cartesian_set.test(cond.var, cond.value) &&
                           !some_possible_effect_in_var[cond.var]) {
                    triggered_for_sure = false;
                    possibly_triggered = false;
                    break;
                }
            }
            if (triggered_for_sure) {
                possibly_triggered_in_var[ef.effect.var] = true;
                triggered_for_sure_in_var[ef.effect.var] = true;
            } else if (!possibly_triggered) {
                some_possible_effect_in_var[ef.effect.var] = false;
            } else {
                next_possibly_triggered_effects_queue.push_back(eff_index);
            }
        }
        possibly_triggered_effects_queue = move(next_possibly_triggered_effects_queue);
        if (possibly_triggered_effects_queue.size() == prev_queue_size) {
            // Fix point reached.
            break;
        }
        bool all_possibly_triggered = true;
        for (bool triggered : possibly_triggered_in_var) {
            if (!triggered) {
                all_possibly_triggered = false;
                break;
            }
        }
        if (all_possibly_triggered) {
            break;
        }
    }

    for (int possibly_triggered_eff_index : possibly_triggered_effects_queue) {
        possibly_triggered_in_var[cond_effects[possibly_triggered_eff_index].effect.var] = true;
    }

    return possibly_triggered_in_var;
}

int AbstractState::n_vars() const {
    return cartesian_set.get_num_variables();
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

bool AbstractState::is_applicable(const std::vector<FactPair> &preconditions) const {
    for (const FactPair &precondition : preconditions) {
        if (!contains(precondition.var, precondition.value)) {
            return false;
        }
    }
    return true;
}

bool AbstractState::is_backward_applicable(const vector<unordered_set<int>> &post) const {
    int n_vars = cartesian_set.get_num_variables();
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


bool AbstractState::reach_with_op(const AbstractState &other,
                                  const OperatorProxy &op,
                                  const Abstraction &abs) const {
    int n_vars = cartesian_set.get_num_variables();
    int op_id = op.get_id();
    vector<bool> vars_with_post(n_vars, false);
    const vector<FactPair> &uncond_effects = abs.get_unconditional_effects(op_id);
    for (const FactPair &eff : uncond_effects) {
        if (!other.contains(eff.var, eff.value)) {
            return false;
        }
        vars_with_post[eff.var] = true;
    }
    const vector<CondEffect> &effects = abs.get_conditional_effects(op_id);
    for (const CondEffect &eff : effects) {
        bool satisfied = true;
        for (const FactPair &cond : eff.conds) {
            int pre_in_cond_var = abs.get_precondition_value(op_id, cond.var);
            if (pre_in_cond_var != UNDEFINED) {
                if (pre_in_cond_var != cond.value) {
                    satisfied = false;
                    break;
                }
            } else if (!contains(cond.var, cond.value)) {
                satisfied = false;
                break;
            }
        }
        if (satisfied) {
            if (!other.contains(eff.effect.var, eff.effect.value)) {
                return false;
            }
            vars_with_post[eff.effect.var] = true;
        }
    }

    const vector<FactPair> &preconds = abs.get_preconditions(op_id);
    for (const FactPair &pre : preconds) {
        if (!vars_with_post[pre.var] && !other.contains(pre.var, pre.value)) {
            return false;
        }
        vars_with_post[pre.var] = true;
    }

    for (int var = 0; var < n_vars; var++) {
        if (!vars_with_post[var] && !is_subset_of(other, var)) {
            return false;
        }
    }

    return true;
}

bool AbstractState::reach_backwards_with_op(const AbstractState &other,
                                            const OperatorProxy &op,
                                            const Abstraction &abs) const {
    int n_vars = cartesian_set.get_num_variables();
    int op_id = op.get_id();
    vector<bool> fixed_value_vars(n_vars, false);
    const CartesianSet &other_set = other.get_cartesian_set();
    // Preconditions must be satisfied by the current state if no possible
    // effect in them, and the same to satisfy effects conditions.
    // Variables with precondition must have precondition value,
    // variables on always fired effects can have any value,
    // variables on conditional effects can have any value if conditions are
    // satisfied, the rest of variables must intersect with this state.
    const vector<FactPair> &preconds = abs.get_preconditions(op_id);
    for (const FactPair &pre : preconds) {
        if (!other_set.test(pre.var, pre.value)) {
            return false;
        }
        fixed_value_vars[pre.var] = true;
    }
    const vector<FactPair> &uncond_effects = abs.get_unconditional_effects(op_id);
    for (const FactPair &eff : uncond_effects) {
        fixed_value_vars[eff.var] = true;
    }
    // To do this call without errors because of non-induced transitions, it
    // should be used during rewiring and doing so is too expensive.
    // vector<bool> possible_effect_in_var = get_possibly_triggered_effect_in_variable(op, abs, optional(ref(other_set)));
    vector<bool> possible_effect_in_var = abs.exists_effect_in_var(op_id);

    const vector<CondEffect> &cond_effects = abs.get_conditional_effects(op_id);
    for (const CondEffect &eff : cond_effects) {
        if (!fixed_value_vars[eff.effect.var] && cartesian_set.test(eff.effect.var, eff.effect.value)) {
            bool conds_satisfied = true;
            for (const FactPair &cond : eff.conds) {
                if (!other_set.test(cond.var, cond.value) ||
                    (!possible_effect_in_var[cond.var] &&
                     !cartesian_set.test(cond.var, cond.value))) {
                    conds_satisfied = false;
                    break;
                }
            }
            if (conds_satisfied) {
                // Any value is good.
                fixed_value_vars[eff.effect.var] = true;
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

void AbstractState::progress(const OperatorProxy &op, const Abstraction &abs) {
    int op_id = op.get_id();
    const vector<FactPair> &preconds = abs.get_preconditions(op_id);
    for (const FactPair &pre : preconds) {
        cartesian_set.set_single_value(pre.var, pre.value);
    }

    // Conditional effects cannot be applied until conditions of all effects have been checked.
    int n_vars = cartesian_set.get_num_variables();
    vector<int> effect_in_var(n_vars, UNDEFINED);
    const vector<CondEffect> &cond_effects = abs.get_conditional_effects(op_id);
    for (const CondEffect &eff : cond_effects) {
        bool satisfied = true;
        for (const FactPair &cond : eff.conds) {
            if (!contains(cond.var, cond.value)) {
                satisfied = false;
                break;
            }
        }
        if (satisfied) {
            effect_in_var[eff.effect.var] = eff.effect.value;
        }
    }
    for (int var = 0; var < n_vars; var++) {
        if (effect_in_var[var] != UNDEFINED) {
            cartesian_set.set_single_value(var, effect_in_var[var]);
        }
    }
    const vector<FactPair> &uncond_effects = abs.get_unconditional_effects(op_id);
    for (const FactPair &eff : uncond_effects) {
        cartesian_set.set_single_value(eff.var, eff.value);
    }
}

void AbstractState::regress(const OperatorProxy &op, const Abstraction &abs) {
    int op_id = op.get_id();

    // To do this call without errors because of non-induced transitions, it
    // should be used during rewiring and doing so is too expensive.
    // bool possibly_triggered_computed = false;
    vector<bool> possibly_triggered_effect_in_variable = abs.exists_effect_in_var(op_id);
    const vector<CondEffect> &cond_effects = abs.get_conditional_effects(op_id);
    for (const CondEffect &eff : cond_effects) {
        // For conditional effects, or the predecessor has this value or the
        // conditions of the effect are satisfied. This is not Cartesian,
        // but we over-approximate it to the Cartesian set that satisfies both.
        if (cartesian_set.test(eff.effect.var, eff.effect.value)) {
            // if (!possibly_triggered_computed) {
            //     possibly_triggered_effect_in_variable = get_possibly_triggered_effect_in_variable(op, abs);
            //     possibly_triggered_computed = true;
            // }
            // // Only effects true in this state are taken into account
            // (not fired effects must not be taken into account).
            // Also, only possibly triggered effects should be considered.
            // For each condition (all effect conditions are assumed to be
            // non-conflicting with the preconditions):
            // 1. It is satisfied in this state.
            // 2. It is not satisfied in this state but another effect has
            //    possibly been triggered in such variable.
            bool possibly_triggered = true;
            for (const FactPair &cond : eff.conds) {
                if (!cartesian_set.test(cond.var, cond.value) &&
                    !possibly_triggered_effect_in_variable[cond.var]) {
                    possibly_triggered = false;
                    break;
                }
            }
            if (possibly_triggered) {
                cartesian_set.add_all(eff.effect.var);
                for (const FactPair &cond : eff.conds) {
                    cartesian_set.add(cond.var, cond.value);
                }
            }
        }
    }
    const vector<FactPair> &uncond_effects = abs.get_unconditional_effects(op_id);
    for (const FactPair &eff : uncond_effects) {
        assert(cartesian_set.test(eff.var, eff.value));
        cartesian_set.add_all(eff.var);
    }
    const vector<FactPair> &preconds = abs.get_preconditions(op_id);
    for (const FactPair &pre : preconds) {
        cartesian_set.set_single_value(pre.var, pre.value);
    }
}

void AbstractState::intersect(const AbstractState &other) {
    int n_vars = cartesian_set.get_num_variables();
    for (int var = 0; var < n_vars; var++) {
        for (int value = 0; value < cartesian_set.n_values(var); value++) {
            if (!other.contains(var, value)) {
                cartesian_set.remove(var, value);
            }
        }
    }
}

void AbstractState::undeviate(const AbstractState &mapped) {
    int n_vars = cartesian_set.get_num_variables();
    for (int var = 0; var < n_vars; var++) {
        if (!is_subset_of(mapped, var)) {
            cartesian_set.remove_all(var);
            int n_values = cartesian_set.n_values(var);
            for (int value = 0; value < n_values; value++) {
                if (mapped.contains(var, value)) {
                    cartesian_set.add(var, value);
                }
            }
        }
    }
}

bool AbstractState::intersects(const AbstractState &other) const {
    int n_vars = cartesian_set.get_num_variables();
    for (int var = 0; var < n_vars; var++) {
        if (!intersects(other, var)) {
            return false;
        }
    }

    return true;
}
bool AbstractState::intersects(const AbstractState &other, int var) const {
    return cartesian_set.intersects(other.get_cartesian_set(), var);
}
bool AbstractState::is_superset_of(const AbstractState &other) const {
    return cartesian_set.is_superset_of(other.get_cartesian_set());
}
bool AbstractState::is_superset_of(const AbstractState &other, int var) const {
    return cartesian_set.is_superset_of(other.get_cartesian_set(), var);
}
bool AbstractState::is_subset_of(const AbstractState &other) const {
    return cartesian_set.is_subset_of(other.get_cartesian_set());
}
bool AbstractState::is_subset_of(const AbstractState &other, int var) const {
    return cartesian_set.is_subset_of(other.get_cartesian_set(), var);
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

bool AbstractState::includes_any(int var, const unordered_set<int> &values) const {
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
    CartesianSet &&trivial_cartesian_set) {
    return make_unique<AbstractState>(0, 0, move(trivial_cartesian_set));
}
}
