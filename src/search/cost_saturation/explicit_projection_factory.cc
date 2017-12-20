#include "explicit_projection_factory.h"

#include "explicit_abstraction.h"
#include "types.h"

#include "../algorithms/ordered_set.h"
#include "../pdbs/match_tree.h"
#include "../task_utils/task_properties.h"
#include "../utils/collections.h"
#include "../utils/logging.h"
#include "../utils/math.h"
#include "../utils/memory.h"

#include <unordered_map>

using namespace std;

namespace cost_saturation {
static int get_pattern_index(const pdbs::Pattern &pattern, int var_id) {
    for (size_t pattern_index = 0; pattern_index < pattern.size(); ++pattern_index) {
        if (pattern[pattern_index] == var_id) {
            return pattern_index;
        }
    }
    return -1;
}

static vector<FactPair> get_relevant_conditions(
    const ConditionsProxy &conditions, const pdbs::Pattern &pattern) {
    vector<FactPair> relevant_conditions;
    for (FactProxy fact : conditions) {
        int var_id = fact.get_variable().get_id();
        int pattern_index = get_pattern_index(pattern, var_id);
        if (pattern_index != -1) {
            relevant_conditions.emplace_back(pattern_index, fact.get_value());
        }
    }
    return relevant_conditions;
}


struct ProjectedEffect {
    const vector<FactPair> relevant_conditions;
    const FactPair fact;
    const bool all_conditions_are_relevant;

    ProjectedEffect(
        const FactPair &projected_fact,
        const EffectConditionsProxy &conditions,
        const pdbs::Pattern &pattern)
        : relevant_conditions(get_relevant_conditions(conditions, pattern)),
          fact(projected_fact),
          all_conditions_are_relevant(conditions.size() == relevant_conditions.size()) {
}
};

class StateMap {
    const pdbs::Pattern pattern;
    const std::vector<int> hash_multipliers;
public:
    StateMap(const pdbs::Pattern &pattern, std::vector<int> &&hash_multipliers)
        : pattern(pattern),
          hash_multipliers(move(hash_multipliers)) {
    }

    int operator()(const State &state) const {
        assert(pattern.size() == hash_multipliers.size());
        int index = 0;
        for (size_t i = 0; i < pattern.size(); ++i) {
            index += hash_multipliers[i] * state[pattern[i]].get_value();
        }
        return index;
    }
};


static vector<vector<FactPair>> get_relevant_preconditions_by_operator(
    const OperatorsProxy &ops, const pdbs::Pattern &pattern) {
    vector<vector<FactPair>> preconditions_by_operator;
    preconditions_by_operator.reserve(ops.size());
    for (OperatorProxy op : ops) {
        preconditions_by_operator.push_back(
            get_relevant_conditions(op.get_preconditions(), pattern));
    }
    return preconditions_by_operator;
}


ExplicitProjectionFactory::ExplicitProjectionFactory(
    const TaskProxy &task_proxy, const pdbs::Pattern &pattern)
    : task_proxy(task_proxy),
      pattern(pattern),
      pattern_size(pattern.size()),
      num_operators(task_proxy.get_operators().size()),
      relevant_preconditions(
        get_relevant_preconditions_by_operator(task_proxy.get_operators(), pattern)) {
    assert(utils::is_sorted_unique(pattern));

    VariablesProxy variables = task_proxy.get_variables();
    variable_to_pattern_index.resize(variables.size(), -1);
    for (size_t i = 0; i < pattern.size(); ++i) {
        variable_to_pattern_index[pattern[i]] = i;
    }

    domain_sizes.reserve(pattern.size());
    for (int var_id : pattern) {
        domain_sizes.push_back(variables[var_id].get_domain_size());
    }

    num_states = 1;
    hash_multipliers.reserve(pattern.size());
    for (int domain_size : domain_sizes) {
        hash_multipliers.push_back(num_states);
        if (utils::is_product_within_limit(
            num_states, domain_size, numeric_limits<int>::max())) {
            num_states *= domain_size;
        } else {
            cerr << "Given pattern is too large! (Overflow occured): " << endl;
            cerr << pattern << endl;
            utils::exit_with(utils::ExitCode::CRITICAL_ERROR);
        }
    }

    compute_transitions();
    goal_states = compute_goal_states();
}

vector<int> ExplicitProjectionFactory::compute_goal_states() const {
    vector<int> goal_states;

    // compute abstract goal var-val pairs
    vector<FactPair> abstract_goals;
    for (FactProxy goal : task_proxy.get_goals()) {
        int var_id = goal.get_variable().get_id();
        int val = goal.get_value();
        if (variable_to_pattern_index[var_id] != -1) {
            abstract_goals.emplace_back(variable_to_pattern_index[var_id], val);
        }
    }

    VariablesProxy variables = task_proxy.get_variables();
    for (int state_index = 0; state_index < num_states; ++state_index) {
        if (is_goal_state(state_index, abstract_goals, variables)) {
            goal_states.push_back(state_index);
        }
    }

    return goal_states;
}

int ExplicitProjectionFactory::rank(const UnrankedState &state) const {
    int index = 0;
    for (int i = 0; i < pattern_size; ++i) {
        index += hash_multipliers[i] * state[i];
    }
    return index;
}

int ExplicitProjectionFactory::unrank(int rank, int pattern_index) const {
    int temp = rank / hash_multipliers[pattern_index];
    return temp % domain_sizes[pattern_index];
}

ExplicitProjectionFactory::UnrankedState ExplicitProjectionFactory::unrank(int rank) const {
    UnrankedState values;
    values.reserve(pattern.size());
    for (int pattern_index = 0; pattern_index < pattern_size; ++pattern_index) {
        values.push_back(unrank(rank, pattern_index));
    }
    return values;
}

vector<ProjectedEffect> ExplicitProjectionFactory::get_projected_effects(
    const OperatorProxy &op) const {
    vector<ProjectedEffect> projected_effects;
    for (EffectProxy effect : op.get_effects()) {
        FactPair effect_fact = effect.get_fact().get_pair();
        int pattern_index = variable_to_pattern_index[effect_fact.var];
        if (pattern_index != -1) {
            projected_effects.emplace_back(
                FactPair(pattern_index, effect_fact.value), effect.get_conditions(), pattern);
        }
    }
    return projected_effects;
}

bool ExplicitProjectionFactory::conditions_are_satisfied(
    const vector<FactPair> &conditions, const UnrankedState &state_values) const {
    for (const FactPair &precondition : conditions) {
        if (state_values[precondition.var] != precondition.value) {
            return false;
        }
    }
    return true;
}

bool ExplicitProjectionFactory::is_applicable(UnrankedState &state_values, int op_id) const {
    return conditions_are_satisfied(relevant_preconditions[op_id], state_values);
}

void ExplicitProjectionFactory::add_transitions(
    const UnrankedState &src_values, int src_rank,
    int op_id, const vector<ProjectedEffect> &effects) {
    UnrankedState definite_dest_values = src_values;
    vector<FactPair> possible_effects;
    for (const ProjectedEffect &effect : effects) {
        if (definite_dest_values[effect.fact.var] != effect.fact.value &&
            conditions_are_satisfied(effect.relevant_conditions, src_values)) {
            if (effect.all_conditions_are_relevant) {
                definite_dest_values[effect.fact.var] = effect.fact.value;
            } else {
                possible_effects.push_back(effect.fact);
            }
        }
    }
    // Apply all subsets of possible effects and add transitions.
    int powerset_size = 1 << possible_effects.size();
    for (int mask = 0; mask < powerset_size; ++mask) {
        UnrankedState possible_dest_values = definite_dest_values;
        for (size_t i = 0; i < possible_effects.size(); ++i) {
            if (mask & (1 << i)) {
                const FactPair &fact = possible_effects[i];
                possible_dest_values[fact.var] = fact.value;
            }
        }
        int dest_rank = rank(possible_dest_values);
        if (dest_rank == src_rank) {
            looping_operators.insert(op_id);
        } else {
            backward_graph[dest_rank].emplace_back(op_id, src_rank);
        }
    }
}

void ExplicitProjectionFactory::compute_transitions() {
    vector<vector<ProjectedEffect>> effects;
    effects.reserve(num_operators);
    for (OperatorProxy op : task_proxy.get_operators()) {
        effects.push_back(get_projected_effects(op));
    }

    backward_graph.resize(num_states);
    for (int src_rank = 0; src_rank < num_states; ++src_rank) {
        vector<int> src_values = unrank(src_rank);
        for (int op_id = 0; op_id < num_operators; ++op_id) {
            if (is_applicable(src_values, op_id)) {
                add_transitions(src_values, src_rank, op_id, effects[op_id]);
            }
        }
    }
}

bool ExplicitProjectionFactory::is_goal_state(
    int state_index,
    const vector<FactPair> &abstract_goals,
    const VariablesProxy &variables) const {
    for (const FactPair &abstract_goal : abstract_goals) {
        int pattern_var_id = abstract_goal.var;
        int var_id = pattern[pattern_var_id];
        VariableProxy var = variables[var_id];
        int temp = state_index / hash_multipliers[pattern_var_id];
        int val = temp % var.get_domain_size();
        if (val != abstract_goal.value) {
            return false;
        }
    }
    return true;
}

unique_ptr<Abstraction> ExplicitProjectionFactory::convert_to_abstraction() {
    return utils::make_unique_ptr<ExplicitAbstraction>(
        StateMap(pattern, move(hash_multipliers)),
        move(backward_graph),
        looping_operators.pop_as_vector(),
        move(goal_states),
        num_operators);
}
}
