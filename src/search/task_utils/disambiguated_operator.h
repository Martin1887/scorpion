#ifndef TASK_UTILS_DISAMBIGUATED_OPERATOR_H
#define TASK_UTILS_DISAMBIGUATED_OPERATOR_H

#include "cartesian_state.h"

#include "../task_proxy.h"

#include <vector>

using namespace cartesian_state;

namespace disambiguation {
class DisambiguatedOperator {
private:
    OperatorProxy op;

    CartesianState precondition;
    std::vector<FactPair> effect;
public:
    DisambiguatedOperator(TaskProxy task, const OperatorProxy &_op);

    void set_spurious();

    bool is_redundant() const;

    // Return if the operator has an effect over var (it should not have
    // effect if the value is a prevail).
    bool has_effect(int variable) const;

    const CartesianState &get_precondition() const;

    const std::vector<FactPair> &get_effect() const;
};
}
#endif
