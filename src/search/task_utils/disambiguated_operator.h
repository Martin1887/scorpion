#ifndef TASK_UTILS_DISAMBIGUATED_OPERATOR_H
#define TASK_UTILS_DISAMBIGUATED_OPERATOR_H

#include "cartesian_state.h"

#include "../task_proxy.h"
#include "disambiguation_method.h"
#include "mutex_information.h"

#include <vector>

using namespace cartesian_state;

namespace disambiguation {
class DisambiguatedOperator {
private:
    OperatorProxy op;

    CartesianState precondition;
    std::vector<int> effect_per_var;
    // TODO: Effects as a Cartesian set with multiple values per var.
    std::vector<FactPair> effects;

    void disambiguate_effects(const EffectsProxy &ep,
                              std::shared_ptr<DisambiguationMethod> &method,
                              std::shared_ptr<MutexInformation> &mutex_information);
    void set_effect_value(int var, int value);
public:
    DisambiguatedOperator(TaskProxy task,
                          const OperatorProxy &_op,
                          std::shared_ptr<DisambiguationMethod> &method,
                          std::shared_ptr<MutexInformation> &mutex_information);

    void set_spurious();

    bool is_redundant() const;

    int get_id() const;

    const std::string get_name() const;

    int get_cost() const;

    bool is_axiom() const;

    const OperatorProxy get_operator() const;

    // Return if the operator has an effect over var (it should not have
    // effect if the value is a prevail).
    bool has_effect(int variable) const;

    int get_var_effect(int variable) const;

    const CartesianState &get_precondition() const;

    const std::vector<FactPair> &get_effects() const;
};
}
#endif
