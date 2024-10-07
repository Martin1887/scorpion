#include "disambiguated_operator.h"

#include "cartesian_set_facts_proxy_iterator.h"

#include <algorithm>

using namespace cartesian_set;
using namespace cartesian_state;
using namespace std;


namespace disambiguation {
void DisambiguatedOperator::disambiguate_effects(const EffectsProxy &ep,
                                                 shared_ptr<DisambiguationMethod> &method,
                                                 shared_ptr<MutexInformation> &mutex_information) {
    CartesianSet pre = precondition.get_cartesian_set();
    CartesianSet effects_cartesian_set(pre);
    for (auto &&ef : ep) {
        FactPair fact = ef.get_fact().get_pair();
        effects_cartesian_set.set_single_value(fact.var, fact.value);
    }
    post.set_cartesian_set(move(effects_cartesian_set));
    method->disambiguate(post, *mutex_information);
    const CartesianSet &post_set = post.get_cartesian_set();
    int n_vars = post_set.get_n_vars();
    for (int var = 0; var < n_vars; var++) {
        if (post.count(var) == 1) {
            effect_in_var[var] = (*post_set.iter(var).begin()).value;
        }
    }
}

DisambiguatedOperator::DisambiguatedOperator(TaskProxy task,
                                             const OperatorProxy &_op,
                                             shared_ptr<DisambiguationMethod> &method,
                                             shared_ptr<MutexInformation> &mutex_information)
    : op(_op),
      precondition(CartesianSet(task, op.get_preconditions())),
      // Empty CartesianSets because they are set in disambiguate_effects after
      // desambiguating preconditions, the initialization here is required by C++.
      post(CartesianSet({})),
      effect_in_var(task.get_variables().size(), -1) {
    method->disambiguate(precondition, *mutex_information);
    disambiguate_effects(op.get_effects(), method, mutex_information);
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
