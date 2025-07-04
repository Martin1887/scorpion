#include "dead_pairs_pruning.h"

#include "../plugins/plugin.h"
#include "../task_utils/task_properties.h"
#include "../utils/logging.h"

using namespace std;

namespace dead_pairs_pruning {
DeadPairsPruning::DeadPairsPruning(const plugins::Options &opts)
    : PruningMethod(opts),
      mutex_facts_in_var_per_eff_per_op() {
}
void DeadPairsPruning::initialize(const std::shared_ptr<AbstractTask> &task) {
    PruningMethod::initialize(task);
    mutex_information = task->mutex_information();

    TaskProxy task_proxy(*task);
    mutex_facts_in_var_per_eff_per_op.reserve(task->get_num_operators());
    for (const OperatorProxy &op : task_proxy.get_operators()) {
        vector<bool> effect_in_var(task->get_num_variables(), false);
        for (const EffectProxy &eff : op.get_effects()) {
            FactPair fact = eff.get_fact().get_pair();
            effect_in_var[fact.var] = true;
        }
        vector<vector<pair<int, set<int>>>> op_mutexes{};
        for (const EffectProxy &eff : op.get_effects()) {
            vector<pair<int, set<int>>> per_var_mutexes{};
            unordered_map<int, int> inserted_vars{};
            const set<FactPair> &mutexes = mutex_information.get_mutexes(eff.get_fact().get_pair());
            for (const FactPair &mutex : mutexes) {
                if (!effect_in_var[mutex.var]) {
                    if (inserted_vars.contains(mutex.var)) {
                        per_var_mutexes[inserted_vars[mutex.var]].second.insert(mutex.value);
                    } else {
                        per_var_mutexes.push_back({mutex.var, {mutex.value}});
                        inserted_vars.insert({mutex.var, per_var_mutexes.size() - 1});
                    }
                }
            }

            op_mutexes.push_back(move(per_var_mutexes));
        }
        mutex_facts_in_var_per_eff_per_op.push_back(move(op_mutexes));
    }
}

void DeadPairsPruning::prune(const State &state,
                             std::vector<OperatorID> &op_ids) {
    vector<OperatorID> remaining_op_ids;
    remaining_op_ids.reserve(op_ids.size());
    for (OperatorID op_id : op_ids) {
        bool spurious = false;

        for (const vector<pair<int, set<int>>> &eff_mutexes : mutex_facts_in_var_per_eff_per_op[op_id.get_index()]) {
            for (const pair<int, set<int>> &mutexes_in_var : eff_mutexes) {
                if (mutexes_in_var.second.contains(state[mutexes_in_var.first].get_value())) {
                    spurious = true;
                    break;
                }
            }
            if (spurious) {
                break;
            }
        }

        if (!spurious) {
            remaining_op_ids.emplace_back(op_id);
        }
    }
    op_ids.swap(remaining_op_ids);
}

class DeadPairsPruningFeature : public plugins::TypedFeature<PruningMethod, DeadPairsPruning> {
public:
    DeadPairsPruningFeature() : TypedFeature("dead_pairs_pruning") {
        // document_group("");
        document_title("Prune by using dead pairs");
        document_synopsis(
            "Prune operators that would produce a state with dead pairs. ");

        add_pruning_options_to_feature(*this);
    }
};

static plugins::FeaturePlugin<DeadPairsPruningFeature> _plugin;
}
