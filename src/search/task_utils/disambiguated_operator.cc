#include "disambiguated_operator.h"

#include "cartesian_set_facts_proxy_iterator.h"

#include "../tasks/domain_abstracted_task.h"

using namespace cartesian_set;
using namespace cartesian_state;
using namespace std;


namespace disambiguation {
void DisambiguatedOperator::disambiguate(const EffectsProxy &ep,
                                         const shared_ptr<DisambiguationMethod> &method,
                                         const shared_ptr<MutexInformation> &mutex_information) {
    // The following steps are followed for a full disambiguation:
    // 1. Remove from preconditions the facts mutex with effects in variables
    //    without effect.
    // 2. Disambiguate preconditions.
    CartesianSet &pre_set = precondition.get_mutable_cartesian_set();
    for (auto &&ef : ep) {
        FactPair fact = ef.get_fact().get_pair();
        effect_in_var[fact.var] = fact.value;
    }

    int n_vars = pre_set.get_n_vars();
    for (int var = 0; var < n_vars; var++) {
        // For variables without effect.
        if (effect_in_var[var] == MULTIPLE_POSTCONDITIONS) {
            // Check if any value of the precondition is mutex with any effect.
            for (int effect_var = 0; effect_var < n_vars; effect_var++) {
                if (effect_in_var[effect_var] != MULTIPLE_POSTCONDITIONS) {
                    int n_values = pre_set.var_size(var);
                    for (int value = 0; value < n_values; value++) {
                        if (mutex_information->are_facts_mutex({var, value}, {effect_var, effect_in_var[effect_var]})) {
                            pre_set.remove(var, value);
                        }
                    }
                }
            }
        }
    }
    method->disambiguate(precondition, *mutex_information);

    // All single-possible-value prevails are actual effects.
    for (int var = 0; var < n_vars; var++) {
        if (effect_in_var[var] == MULTIPLE_POSTCONDITIONS && precondition.count(var) == 1) {
            effect_in_var[var] = (*pre_set.iter(var).begin()).value;
        }
    }
}

DisambiguatedOperator::DisambiguatedOperator(TaskProxy task,
                                             const OperatorProxy &_op,
                                             const shared_ptr<DisambiguationMethod> &method,
                                             const shared_ptr<MutexInformation> &mutex_information)
    : op(_op),
      precondition(CartesianSet(task, op.get_preconditions())),
      effect_in_var(task.get_variables().size(), MULTIPLE_POSTCONDITIONS) {
    disambiguate(op.get_effects(), method, mutex_information);
}

DisambiguatedOperator::DisambiguatedOperator(CartesianSet &&_pre,
                                             const OperatorProxy &_op)
    : op(_op),
      precondition(move(_pre)),
      effect_in_var(precondition.get_cartesian_set().get_n_vars(), MULTIPLE_POSTCONDITIONS) {
    int n_vars = precondition.get_cartesian_set().get_n_vars();
    const CartesianSet &pre_set = precondition.get_cartesian_set();
    // All postconditions with a single value are actual effects.
    for (int var = 0; var < n_vars; var++) {
        if (effect_in_var[var] == MULTIPLE_POSTCONDITIONS && precondition.count(var) == 1) {
            effect_in_var[var] = (*pre_set.iter(var).begin()).value;
        }
    }
}


bool DisambiguatedOperator::is_redundant() const {
    return precondition.is_spurious();
}

int DisambiguatedOperator::get_id() const {
    return op.get_id();
}

const string DisambiguatedOperator::get_name() const {
    return op.get_name();
}

int DisambiguatedOperator::get_cost() const {
    return op.get_cost();
}

bool DisambiguatedOperator::is_axiom() const {
    return op.is_axiom();
}

const OperatorProxy DisambiguatedOperator::get_operator() const {
    return op;
}

const CartesianState &DisambiguatedOperator::get_precondition() const {
    return precondition;
}

CartesianSet DisambiguatedOperator::get_post_cartesian_set() const {
    CartesianSet post = precondition.get_cartesian_set();
    int n_vars = effect_in_var.size();
    for (int var = 0; var < n_vars; var++) {
        if (effect_in_var[var] != MULTIPLE_POSTCONDITIONS) {
            post.set_single_value(var, effect_in_var[var]);
        }
    }

    return post;
}

bool DisambiguatedOperator::has_effect(int var) const {
    return effect_in_var[var] != MULTIPLE_POSTCONDITIONS;
}
int DisambiguatedOperator::get_effect(int var) const {
    return effect_in_var[var];
}
}
