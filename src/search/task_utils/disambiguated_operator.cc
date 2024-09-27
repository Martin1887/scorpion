#include "disambiguated_operator.h"

#include "cartesian_set_facts_proxy_iterator.h"

#include <algorithm>

using namespace cartesian_set;
using namespace cartesian_state;
using namespace std;


namespace disambiguation {
void DisambiguatedOperator::set_effect_value(int var, int value) {
    effects.push_back(FactPair {var, value});
    effect_per_var[var] = value;
}

void DisambiguatedOperator::disambiguate_effects(const EffectsProxy &ep,
                                                 shared_ptr<DisambiguationMethod> &method,
                                                 shared_ptr<MutexInformation> &mutex_information) {
    // Effects are set as a single value over precondition's CartesianSet.
    // If any variable ends with a single value, it is an actual effect.
    CartesianSet effects_cartesian_set(precondition.get_cartesian_set());
    for (auto &&ef : ep) {
        effects_cartesian_set.set_single_value(ef.get_fact().get_variable().get_id(),
                                               ef.get_fact().get_value());
    }
    CartesianState ef_cartesian_state(move(effects_cartesian_set));
    method->disambiguate(ef_cartesian_state, *mutex_information);
    const CartesianSet &ef_set = ef_cartesian_state.get_cartesian_set();
    int n_vars = ef_set.n_vars();
    for (int var = 0; var < n_vars; var++) {
        if (ef_set.count(var) == 1) {
            set_effect_value(var, (*ef_set.iter(var).begin()).value);
        }
    }
}

DisambiguatedOperator::DisambiguatedOperator(TaskProxy task,
                                             const OperatorProxy &_op,
                                             shared_ptr<DisambiguationMethod> &method,
                                             shared_ptr<MutexInformation> &mutex_information)
    : op(_op),
      precondition(CartesianSet(task, op.get_preconditions())),
      effect_per_var(precondition.get_cartesian_set().n_vars(), -1),
      effects() {
    method->disambiguate(precondition, *mutex_information);
    disambiguate_effects(op.get_effects(), method, mutex_information);
}


bool DisambiguatedOperator::is_redundant() const {
    return precondition.is_spurious() || effects.empty();
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

const vector<FactPair> &DisambiguatedOperator::get_effects() const {
    return effects;
}

bool DisambiguatedOperator::has_effect(int variable) const {
    return effect_per_var[variable] != -1;
}

int DisambiguatedOperator::get_var_effect(int variable) const {
    return effect_per_var[variable];
}

void DisambiguatedOperator::set_spurious() {
    effects.clear();
}
}
