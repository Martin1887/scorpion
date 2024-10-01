#include "flaw_search.h"

#include "abstraction.h"
#include "abstract_state.h"
#include "flaw.h"
#include "shortest_paths.h"
#include "split_selector.h"
#include "transition_system.h"
#include "utils.h"

#include "../task_utils/cartesian_set_facts_proxy_iterator.h"
#include "../task_utils/disambiguated_operator.h"
#include "../task_utils/successor_generator.h"
#include "../task_utils/task_properties.h"
#include "../utils/countdown_timer.h"
#include "../utils/rng.h"

#include <iterator>
#include <locale>

using namespace std;

namespace cartesian_abstractions {
unique_ptr<Split> FlawSearch::create_backward_split(
    const vector<reference_wrapper<const CartesianState>> &states, int abstract_state_id, Cost solution_cost, bool split_unwanted_values) {
    compute_splits_timer.resume();
    const AbstractState &abstract_state = abstraction.get_state(abstract_state_id);

    if (log.is_at_least_debug()) {
        log << endl;
        log << "Create split for abstract state " << abstract_state_id << " and "
            << states.size() << " flaw-search states:" << endl;
        for (CartesianState fss : states) {
            log << fss << endl;
        }
    }

    vector<vector<Split>> splits;
    // Splits are grouped by variable only if split by wanted values.
    if (split_unwanted_values) {
        splits = vector<vector<Split>>();
    } else {
        splits = vector<vector<Split>>(task_proxy.get_variables().size());
    }
    for (auto &pair : get_f_optimal_backward_transitions(abstract_state_id)) {
        if (log.is_at_least_debug()) {
            log << "Optimal backward transition(s): " << pair.first << ", "
                << pair.second << endl;
        }
        int op_id = pair.first;
        const vector<int> &sources = pair.second;
        const disambiguation::DisambiguatedOperator &op = (*abstraction.get_transition_system().get_operators())[op_id];
        vector<int> eff_values(domain_sizes.size(), -1);
        for (const FactPair &ef : op.get_effects()) {
            eff_values[ef.var] = ef.value;
        }
        // prevail conditions
        for (const auto &&[cond_var, cond_value] : op.get_precondition().get_cartesian_set().iter()) {
            if (eff_values[cond_var] == -1) {
                eff_values[cond_var] = cond_value;
            }
        }

        if (log.is_at_least_debug()) {
            log << "Operator: " << op.get_name() << endl;
        }

        vector<bool> applicable(states.size(), true);
        vector<vector<int>> state_value_count{};
        for (size_t var = 0; var < domain_sizes.size(); var++) {
            state_value_count.push_back(vector<int>(domain_sizes[var], 0));
        }

        for (size_t i = 0; i < states.size(); ++i) {
            const CartesianState &state = states[i];
            const CartesianSet &state_cartesian_set = state.get_cartesian_set();
            for (int not_applicable : state.vars_not_backward_applicable(op)) {
                if (log.is_at_least_debug()) {
                    log << "Not applicable!" << endl;
                    log << "State: " << state << ", var: " << not_applicable << endl;
                }
                // Applicability flaw
                applicable[i] = false;
                for (const auto &&[fact_var, fact_value] : state_cartesian_set.iter(not_applicable)) {
                    if (abstract_state.includes(not_applicable, fact_value)) {
                        ++state_value_count[not_applicable][fact_value];
                    }
                }
            }
        }
        for (size_t var = 0; var < domain_sizes.size(); var++) {
            int eff_value = eff_values[var];
            for (int value = 0; value < domain_sizes[var]; ++value) {
                if (state_value_count[var][value] > 0) {
                    assert(value != eff_value);
                    if (log.is_at_least_debug()) {
                        log << "add_split(var " << var << ", val " << value
                            << "!=" << eff_value << ", state_value_count: "
                            << state_value_count[var][value] << ")" << endl;
                    }
                    if (split_unwanted_values) {
                        add_split(splits, Split(
                                      abstract_state_id, var, eff_value,
                                      {value}, state_value_count[var][value],
                                      op.get_cost()), true);
                    } else {
                        add_split(splits, Split(
                                      abstract_state_id, var, value,
                                      {eff_value}, state_value_count[var][value],
                                      op.get_cost()));
                    }
                }
            }
        }

        phmap::flat_hash_map<int, vector<reference_wrapper<const CartesianState>>> deviation_states_by_source;
        for (size_t i = 0; i < states.size(); ++i) {
            if (!applicable[i] /*&& !in_sequence*/) {
                if (log.is_at_least_debug()) {
                    log << "Not applicable" << endl;
                }
                continue;
            }
            const CartesianState &state = states[i];
            if (!in_sequence) {
                assert(state.is_backward_applicable(op));
            }
            bool source_hit = false;
            for (int source : sources) {
                if (!utils::extra_memory_padding_is_reserved()) {
                    return nullptr;
                }

                // At most one of the f-optimal targets can include the successor state.
                if (!source_hit &&
                    ((applicable[i] && state.reach_backwards_with_op(abstraction.get_state(source), op)) ||
                     (!applicable[i] && state.reach_backwards_with_inapplicable_op(abstraction.get_state(source), op)))) {
                    // No flaw
                    source_hit = true;
                    if (log.is_at_least_debug()) {
                        log << "source_hit, state: " << state << ", source: "
                            << source << endl;
                        log << "source: " << abstraction.get_state(source) << endl;
                        log << "state: " << state << endl;
                    }
                } else {
                    // Deviation flaw
                    if (log.is_at_least_debug()) {
                        log << "Deviation states by source, state: " << state
                            << ", source: " << source << endl;
                    }
                    deviation_states_by_source[source].push_back(ref(state));
                }
            }
        }

        for (auto &&[source, deviation_states] : deviation_states_by_source) {
            if (!deviation_states.empty()) {
                int num_vars = domain_sizes.size();
                get_deviation_splits(
                    abstract_state, deviation_states,
                    get_unaffected_variables(op, num_vars),
                    abstraction.get_state(source), domain_sizes, op.get_cost(),
                    splits, split_unwanted_values);
            }
        }
    }

    int num_splits = 0;
    for (auto &var_splits : splits) {
        num_splits += var_splits.size();
    }
    if (log.is_at_least_debug()) {
        log << "Unique splits: " << num_splits << endl;
    }
    compute_splits_timer.stop();

    if (num_splits == 0) {
        return nullptr;
    }

    pick_split_timer.resume();
    Split split = split_selector.pick_split(abstract_state, move(splits), solution_cost, rng);
    pick_split_timer.stop();
    return utils::make_unique_ptr<Split>(move(split));
}

unique_ptr<Split> FlawSearch::create_backward_split_from_init_state(
    const vector<reference_wrapper<const CartesianState>> &states, int abstract_state_id, Cost solution_cost, bool split_unwanted_values) {
    compute_splits_timer.resume();
    const AbstractState &abstract_state = abstraction.get_state(abstract_state_id);

    if (log.is_at_least_debug()) {
        log << endl;
        log << "Create split for abstract state " << abstract_state_id << " and "
            << states.size() << " flaw-search states:" << endl;
        for (CartesianState fss : states) {
            log << fss << endl;
        }
    }

    const State init_state = task_proxy.get_initial_state();
    vector<vector<Split>> splits;
    // Splits are grouped by variable only if split by wanted values.
    if (split_unwanted_values) {
        splits = vector<vector<Split>>();
    } else {
        splits = vector<vector<Split>>(task_proxy.get_variables().size());
    }
    int num_vars = (int)domain_sizes.size();
    for (int var = 0; var < num_vars; var++) {
        if (abstract_state.count(var) > 1) {
            int init_value = init_state[var].get_value();

            if (split_unwanted_values) {
                for (CartesianState state : states) {
                    if (!state.includes(var, init_value)) {
                        for (const auto &&[fact_var, fact_value] : state.get_cartesian_set().iter(var)) {
                            if (abstract_state.includes(var, fact_value)) {
                                if (log.is_at_least_debug()) {
                                    log << "add_split(var " << var << ", val " << fact_value
                                        << "!=" << init_value << ")" << endl;
                                }
                                add_split(splits, Split(
                                              abstract_state_id, var, init_value,
                                              {fact_value}, 1), true);
                            }
                        }
                    }
                }
            } else {
                vector<int> other_values{};
                for (int value = 0; value < domain_sizes[var]; value++) {
                    if (value != init_value && abstract_state.includes(var, value)) {
                        other_values.push_back(value);
                    }
                }
                if (log.is_at_least_debug()) {
                    log << "add_split(var " << var << ", val " << init_value
                        << "!=" << other_values << ")" << endl;
                }
                add_split(splits, Split(
                              abstract_state_id, var, init_value,
                              move(other_values), 1));
            }
        }
    }

    int num_splits = 0;
    for (auto &var_splits : splits) {
        num_splits += var_splits.size();
    }
    if (log.is_at_least_debug()) {
        log << "Unique splits: " << num_splits << endl;
    }
    compute_splits_timer.stop();

    if (num_splits == 0) {
        return nullptr;
    }

    pick_split_timer.resume();
    Split split = split_selector.pick_split(abstract_state, move(splits), solution_cost, rng);
    pick_split_timer.stop();
    return utils::make_unique_ptr<Split>(move(split));
}
}
