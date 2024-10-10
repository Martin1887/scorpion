#include "disambiguated_operator.h"

#include "cartesian_set_facts_proxy_iterator.h"

#include <algorithm>

using namespace cartesian_set;
using namespace cartesian_state;
using namespace std;


namespace disambiguation {
void DisambiguatedOperator::disambiguate(const EffectsProxy &ep,
                                         const shared_ptr<DisambiguationMethod> &method,
                                         const shared_ptr<MutexInformation> &mutex_information) {
    // The following steps are followed for a full disambiguation:
    // 1. Disambiguate postconditions.
    // 2. Assign disambiguated postconditions values of variables without
    //    effect to preconditions.
    // 3. Disambiguate preconditions.
    // 4. Assign disambiguated precondtions values of variables without
    //    effect to postconditions.
    CartesianSet effects_cartesian_set(precondition.get_cartesian_set());
    // Build non-disambiguated postconditions.
    for (auto &&ef : ep) {
        FactPair fact = ef.get_fact().get_pair();
        effects_cartesian_set.set_single_value(fact.var, fact.value);
        // Effects are set at the end of the function, but doing this here allows
        // to know if a variable has no effect using effect_in_var.
        effect_in_var[fact.var] = fact.value;
    }
    post.set_cartesian_set(move(effects_cartesian_set));
    // Step 1.
    method->disambiguate(post, *mutex_information);
    const CartesianSet &post_set = post.get_cartesian_set();

    // Steps 2. and 3.
    int n_vars = post_set.get_n_vars();
    for (int var = 0; var < n_vars; var++) {
        if (effect_in_var[var] == MULTIPLE_POSTCONDITIONS) {
            precondition.set_var_values(var, post_set);
        }
    }
    method->disambiguate(precondition, *mutex_information);

    // Step 4.
    const CartesianSet &pre_set = precondition.get_cartesian_set();
    for (int var = 0; var < n_vars; var++) {
        if (effect_in_var[var] == MULTIPLE_POSTCONDITIONS) {
            post.set_var_values(var, pre_set);
        }
    }
    method->disambiguate(post, *mutex_information);

    // All postconditions with a single value are actual effects.
    for (int var = 0; var < n_vars; var++) {
        if (post.count(var) == 1) {
            effect_in_var[var] = (*post_set.iter(var).begin()).value;
        }
    }
}

DisambiguatedOperator::DisambiguatedOperator(TaskProxy task,
                                             const OperatorProxy &_op,
                                             const shared_ptr<DisambiguationMethod> &method,
                                             const shared_ptr<MutexInformation> &mutex_information)
    : op(_op),
      precondition(CartesianSet(task, op.get_preconditions())),
      // Empty CartesianSets because they are set in disambiguate_effects after
      // desambiguating preconditions, the initialization here is required by C++.
      post(CartesianSet({})),
      effect_in_var(task.get_variables().size(), -1) {
    disambiguate(op.get_effects(), method, mutex_information);
}


bool DisambiguatedOperator::is_redundant() const {
    return precondition.is_spurious() || post.is_spurious();
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

const CartesianState &DisambiguatedOperator::get_post() const {
    return post;
}

bool DisambiguatedOperator::has_effect(int var) const {
    return effect_in_var[var] != MULTIPLE_POSTCONDITIONS;
}
int DisambiguatedOperator::get_effect(int var) const {
    return effect_in_var[var];
}
}
