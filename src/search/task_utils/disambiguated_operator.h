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
    std::vector<int> effect_in_var;

    void disambiguate(const EffectsProxy &ep,
                      const std::shared_ptr<DisambiguationMethod> &method,
                      const std::shared_ptr<MutexInformation> &mutex_information);
public:
    DisambiguatedOperator(TaskProxy task,
                          const OperatorProxy &_op,
                          const std::shared_ptr<DisambiguationMethod> &method,
                          const std::shared_ptr<MutexInformation> &mutex_information);
    DisambiguatedOperator(CartesianSet &&_pre,
                          std::vector<int> &&_effect_in_var,
                          const OperatorProxy &_op);

    bool is_redundant() const;

    int get_id() const;

    const std::string get_name() const;

    int get_cost() const;

    bool is_axiom() const;

    const OperatorProxy get_operator() const;
    const CartesianState &get_precondition() const;
    CartesianSet get_post_cartesian_set() const;
    const std::vector<int> &get_effect_in_var() const;
    bool has_effect(int var) const;
    int get_effect(int var) const;
};
}
#endif
