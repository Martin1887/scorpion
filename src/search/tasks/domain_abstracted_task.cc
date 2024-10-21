#include "domain_abstracted_task.h"

#include "../task_utils/cartesian_set_facts_proxy_iterator.h"
#include "../task_utils/disambiguated_operator.h"
#include "../utils/system.h"

using namespace std;

namespace extra_tasks {
/*
  If we need the same functionality again in another task, we can move this to
  actract_task.h. We should then document that this method is only supposed to
  be used from within AbstractTasks. More high-level users should use
  has_conditional_effects(TaskProxy) from task_tools.h instead.
*/
static bool has_conditional_effects(const AbstractTask &task) {
    int num_ops = task.get_num_operators();
    for (int op_index = 0; op_index < num_ops; ++op_index) {
        int num_effs = task.get_num_operator_effects(op_index, false);
        for (int eff_index = 0; eff_index < num_effs; ++eff_index) {
            int num_conditions = task.get_num_operator_effect_conditions(
                op_index, eff_index, false);
            if (num_conditions > 0) {
                return true;
            }
        }
    }
    return false;
}

ValueMap::ValueMap(
    const AbstractTask &task,
    const AbstractTask &parent_task,
    vector<vector<int>> &&value_map)
    : variable_to_pool_index(task.get_num_variables(), -1) {
    // Only store value mappings for abstracted variables.
    for (int var = 0; var < task.get_num_variables(); ++var) {
        if (task.get_variable_domain_size(var) < parent_task.get_variable_domain_size(var)) {
            variable_to_pool_index[var] = abstracted_variables.size();
            abstracted_variables.push_back({var, variable_to_pool_index[var]});
            new_values.push_back(move(value_map[var]));
        }
    }
    abstracted_variables.shrink_to_fit();
}

void ValueMap::convert(vector<int> &state_values) const {
    for (const AbstractedVariable &abs_var : abstracted_variables) {
        int old_value = state_values[abs_var.var];
        int new_value = new_values[abs_var.pool_index][old_value];
        state_values[abs_var.var] = new_value;
    }
}

FactPair ValueMap::convert(const FactPair &fact) const {
    if (variable_to_pool_index[fact.var] == -1) {
        // This is the common case.
        return fact;
    } else {
        return FactPair(fact.var, new_values[variable_to_pool_index[fact.var]][fact.value]);
    }
}

bool ValueMap::does_convert_values() const {
    return !abstracted_variables.empty();
}


DomainAbstractedTask::DomainAbstractedTask(
    const shared_ptr<AbstractTask> &parent,
    vector<int> &&_domain_size,
    vector<int> &&initial_state_values,
    vector<FactPair> &&goals,
    vector<vector<string>> &&fact_names,
    vector<vector<int>> &&_value_map,
    const MutexInformation &mutex_information)
    : DelegatingTask(parent),
      domain_size(move(_domain_size)),
      initial_state_values(move(initial_state_values)),
      goals(move(goals)),
      fact_names(move(fact_names)),
      value_map(*this, *parent, move(_value_map)),
      mutexes(mutex_information.convert(domain_size, value_map)) {
    if (parent->get_num_axioms() > 0) {
        ABORT("DomainAbstractedTask doesn't support axioms.");
    }
    if (has_conditional_effects(*parent)) {
        ABORT("DomainAbstractedTask doesn't support conditional effects.");
    }
}

int DomainAbstractedTask::get_variable_domain_size(int var) const {
    return domain_size[var];
}

string DomainAbstractedTask::get_fact_name(const FactPair &fact) const {
    return fact_names[fact.var][fact.value];
}

bool DomainAbstractedTask::are_facts_mutex(const FactPair &fact1, const FactPair &fact2) const {
    return mutexes.are_facts_mutex(fact1, fact2);
}

MutexInformation DomainAbstractedTask::mutex_information() const {
    return mutexes;
}

FactPair DomainAbstractedTask::get_operator_precondition(
    int op_index, int fact_index, bool is_axiom) const {
    return value_map.convert(
        parent->get_operator_precondition(op_index, fact_index, is_axiom));
}

FactPair DomainAbstractedTask::get_operator_effect(
    int op_index, int eff_index, bool is_axiom) const {
    return value_map.convert(
        parent->get_operator_effect(op_index, eff_index, is_axiom));
}

FactPair DomainAbstractedTask::get_goal_fact(int index) const {
    return value_map.convert(parent->get_goal_fact(index));
}

vector<int> DomainAbstractedTask::get_initial_state_values() const {
    return initial_state_values;
}

void DomainAbstractedTask::convert_state_values_from_parent(
    vector<int> &values) const {
    value_map.convert(values);
}

bool DomainAbstractedTask::does_convert_ancestor_state_values(
    const AbstractTask *) const {
    return value_map.does_convert_values();
}

CartesianSet DomainAbstractedTask::convert_cartesian_set(const CartesianSet &cartesian_set) const {
    int n_vars = domain_size.size();
    CartesianSet new_set(domain_size);
    for (int var = 0; var < n_vars; var++) {
        new_set.remove_all(var);
        for (int value = 0; value < domain_size[var]; value++) {
            for (auto &&fact : cartesian_set.iter(var)) {
                if (value_map.convert(fact).value == value) {
                    new_set.add(var, value);
                    break;
                }
            }
        }
    }

    return new_set;
}

disambiguation::DisambiguatedOperator DomainAbstractedTask::convert_disambiguated_operator(const disambiguation::DisambiguatedOperator &op) const {
    CartesianSet new_pre = convert_cartesian_set(op.get_precondition().get_cartesian_set());
    CartesianSet new_post = convert_cartesian_set(op.get_post().get_cartesian_set());

    return disambiguation::DisambiguatedOperator(move(new_pre), move(new_post), op.get_operator());
}
}
