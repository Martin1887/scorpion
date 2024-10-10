#ifndef TASK_UTILS_DISAMBIGUATED_OPERATOR_H
#define TASK_UTILS_DISAMBIGUATED_OPERATOR_H

#include "cartesian_state.h"

#include "../task_proxy.h"
#include "disambiguation_method.h"
#include "mutex_information.h"

#include <vector>

using namespace cartesian_state;

namespace disambiguation {
const int MULTIPLE_POSTCONDITIONS = -1;

class DisambiguatedOperator {
private:
    OperatorProxy op;

    CartesianState precondition;
    CartesianState post;
    std::vector<int> effect_in_var;

    void disambiguate(const EffectsProxy &ep,
                      const std::shared_ptr<DisambiguationMethod> &method,
                      const std::shared_ptr<MutexInformation> &mutex_information);
public:
    DisambiguatedOperator(TaskProxy task,
                          const OperatorProxy &_op,
                          const std::shared_ptr<DisambiguationMethod> &method,
                          const std::shared_ptr<MutexInformation> &mutex_information);

    bool is_redundant() const;

    int get_id() const;

    const std::string get_name() const;

    int get_cost() const;

    bool is_axiom() const;

    const OperatorProxy get_operator() const;
    const CartesianState &get_precondition() const;
    const CartesianState &get_post() const;
    bool has_effect(int var) const;
    int get_effect(int var) const;
};
}
#endif
