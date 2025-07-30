#ifndef PRUNING_DEAD_PAIRS_H
#define PRUNING_DEAD_PAIRS_H

#include "../pruning_method.h"
#include "../task_proxy.h"
#include "../task_utils/mutex_information.h"

class MutexInformation;

namespace dead_pairs_pruning {
class DeadPairsPruning : public PruningMethod {
    MutexInformation mutex_information;
    // Operator -> Effect Index -> Mutex vars(mutex var, Mutex values in mutex var).
    std::vector<std::vector<std::vector<std::pair<int, std::set<int>>>>> mutex_facts_in_var_per_eff_per_op;

    virtual void prune(const State &state,
                       std::vector<OperatorID> &op_ids) override;
public:
    explicit DeadPairsPruning(const plugins::Options &opts);
    virtual void initialize(const std::shared_ptr<AbstractTask> &task) override;
};
}

#endif
