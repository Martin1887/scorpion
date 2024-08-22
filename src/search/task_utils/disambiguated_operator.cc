#include "disambiguated_operator.h"

#include <algorithm>

using namespace cartesian_set;
using namespace cartesian_state;
using namespace std;


namespace disambiguation {
DisambiguatedOperator::DisambiguatedOperator(TaskProxy task, const OperatorProxy &_op)
    : op(_op),
      precondition(CartesianSet(task, op.get_preconditions())) {
}

bool DisambiguatedOperator::is_redundant() const {
    return precondition.is_spurious() || effect.empty();
}

const CartesianState &DisambiguatedOperator::get_precondition() const {
    return precondition;
}

const vector <FactPair> &DisambiguatedOperator::get_effect() const {
    return effect;
}

bool DisambiguatedOperator::has_effect(int variable) const {
    return ranges::find_if(effect, [variable](const auto &fact) {
                               return fact.var == variable;
                           }) != effect.end();
}

void DisambiguatedOperator::set_spurious() {
    effect.clear();
}
}
