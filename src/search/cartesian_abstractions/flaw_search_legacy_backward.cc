#include "flaw_search.h"

#include "abstraction.h"
#include "abstract_state.h"
#include "flaw.h"
#include "shortest_paths.h"
#include "split_selector.h"
#include "transition_system.h"
#include "utils.h"

#include "../task_utils/successor_generator.h"
#include "../task_utils/task_properties.h"
#include "../utils/countdown_timer.h"
#include "../utils/rng.h"

#include <iterator>
#include <locale>

using namespace std;

namespace cartesian_abstractions {
void FlawSearch::get_deviation_backward_splits(
    const AbstractState &abs_state,
    const vector<AbstractState> &flaw_search_states,
    const vector<int> &unaffected_variables,
    const AbstractState &source_abs_state,
    const vector<int> &domain_sizes,
    const int op_cost,
    vector<vector<Split>> &splits,
    bool split_unwanted_values) {
    /*
      For each fact in the flaw-search state that is not contained in the
      source abstract state, loop over all values in the domain of the
      corresponding variable. The values that are in both the current and
      the source abstract state are the "wanted" ones, i.e., the ones that
      we want to split off. This test can be specialized for applicability and
      deviation flaws. Here, we consider deviation flaws.

      Let the desired abstract transition be (a, o, t) and the deviation be
      (a, o, b). We distinguish three cases for each variable v:

      eff(o)[v] defined: no split possible since o is applicable in s.
      eff(o)[v] undefined, pre(o)[v] defined: no split possible since regression adds whole domain.
      eff(o)[v] and pre(o)[v] undefined: if s[v] \notin t[v], wanted = intersect(a[v], b[v]).
    */
    // Note: it could be faster to use an efficient hash map for this.
    vector<vector<int>> fact_count(domain_sizes.size());
    vector<bool> var_fact_count(domain_sizes.size());
    for (size_t var = 0; var < domain_sizes.size(); ++var) {
        fact_count[var].resize(domain_sizes[var], 0);
    }
    for (const AbstractState &flaw_search_st : flaw_search_states) {
        for (int var : unaffected_variables) {
            // When disambiguation is implemented, `contains` will be possible
            // instead of `intersects`
            if (!source_abs_state.domain_subsets_intersect(flaw_search_st, var)) {
                for (int state_value : flaw_search_st.get_cartesian_set().get_values(var)) {
                    if (abs_state.contains(var, state_value)) {
                        ++fact_count[var][state_value];
                        var_fact_count[var] = true;
                    }
                }
            }
        }
    }
    for (size_t var = 0; var < domain_sizes.size(); ++var) {
        // the `wanted` vector is the same for all values of the variable
        vector<int> wanted;
        if (var_fact_count[var]) {
            for (int value = 0; value < domain_sizes[var]; ++value) {
                if (abs_state.contains(var, value) &&
                    source_abs_state.contains(var, value)) {
                    wanted.push_back(value);
                }
            }
            for (int value = 0; value < domain_sizes[var]; ++value) {
                if (fact_count[var][value] && !source_abs_state.contains(var, value)) {
                    assert(!wanted.empty());
                    if (split_unwanted_values) {
                        for (int want : wanted) {
                            FlawSearch::add_split(splits, Split(
                                                      abs_state.get_id(), var, want, {value},
                                                      fact_count[var][value], op_cost), true);
                        }
                    } else {
                        vector<int> wanted_copy(wanted);
                        FlawSearch::add_split(splits, Split(
                                                  abs_state.get_id(), var, value, move(wanted_copy),
                                                  fact_count[var][value], op_cost));
                    }
                }
            }
        }
    }
}

unique_ptr<Split> FlawSearch::create_backward_split(
    const vector<AbstractState> &states, int abstract_state_id, bool split_unwanted_values) {
    compute_splits_timer.resume();
    const AbstractState &abstract_state = abstraction.get_state(abstract_state_id);

    if (log.is_at_least_debug()) {
        log << endl;
        log << "Create split for abstract state " << abstract_state_id << " and "
            << states.size() << " flaw-search states:" << endl;
        for (AbstractState fss : states) {
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
        OperatorProxy op = task_proxy.get_operators()[op_id];
        vector<int> eff_values(domain_sizes.size(), -1);
        for (EffectProxy ef : op.get_effects()) {
            eff_values[ef.get_fact().get_variable().get_id()] =
                ef.get_fact().get_value();
        }
        // prevail conditions
        for (FactProxy cond : op.get_preconditions()) {
            if (eff_values[cond.get_variable().get_id()] == -1) {
                eff_values[cond.get_variable().get_id()] = cond.get_value();
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
            const AbstractState &state = states[i];
            for (int not_applicable : state.vars_not_backward_applicable(op)) {
                if (log.is_at_least_debug()) {
                    log << "Not applicable!" << endl;
                    log << "State: " << state << ", var: " << not_applicable << endl;
                }
                // Applicability flaw
                applicable[i] = false;
                for (int value : state.get_cartesian_set().get_values(not_applicable)) {
                    if (abstract_state.contains(not_applicable, value)) {
                        ++state_value_count[not_applicable][value];
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

        phmap::flat_hash_map<int, vector<AbstractState>> deviation_states_by_source;
        for (size_t i = 0; i < states.size(); ++i) {
            if (!applicable[i]) {
                if (log.is_at_least_debug()) {
                    log << "Not applicable" << endl;
                }
                continue;
            }
            const AbstractState &state = states[i];
            assert(state.is_backward_applicable(op));
            AbstractState succ_state(-1, -1, state.regress(op));
            bool source_hit = false;
            for (int source : sources) {
                if (!utils::extra_memory_padding_is_reserved()) {
                    return nullptr;
                }

                // At most one of the f-optimal targets can include the successor state.
                if (!source_hit && abstraction.get_state(source).intersects(succ_state)) {
                    // No flaw
                    source_hit = true;
                    if (log.is_at_least_debug()) {
                        log << "source_hit, state: " << state << ", source: "
                            << source << endl;
                        log << "source: " << abstraction.get_state(source) << endl;
                        log << "succ_state: " << succ_state << endl;
                        log << "state: " << state << endl;
                    }
                } else {
                    // Deviation flaw
                    if (log.is_at_least_debug()) {
                        log << "Deviation states by source, state: " << state
                            << ", source: " << source << endl;
                    }
                    deviation_states_by_source[source].push_back(state);
                }
            }
        }

        for (auto &pair : deviation_states_by_source) {
            int source = pair.first;
            const vector<AbstractState> &deviation_states = pair.second;
            if (!deviation_states.empty()) {
                int num_vars = domain_sizes.size();
                get_deviation_backward_splits(
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
    Split split = split_selector.pick_split(abstract_state, move(splits), rng);
    pick_split_timer.stop();
    return utils::make_unique_ptr<Split>(move(split));
}

unique_ptr<Split> FlawSearch::create_backward_split_from_init_state(
    const vector<AbstractState> &states, int abstract_state_id, bool split_unwanted_values) {
    compute_splits_timer.resume();
    const AbstractState &abstract_state = abstraction.get_state(abstract_state_id);

    if (log.is_at_least_debug()) {
        log << endl;
        log << "Create split for abstract state " << abstract_state_id << " and "
            << states.size() << " flaw-search states:" << endl;
        for (AbstractState fss : states) {
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
                for (AbstractState state : states) {
                    if (!state.contains(var, init_value)) {
                        for (int state_value : state.get_cartesian_set().get_values(var)) {
                            if (abstract_state.contains(var, state_value)) {
                                if (log.is_at_least_debug()) {
                                    log << "add_split(var " << var << ", val " << state_value
                                        << "!=" << init_value << ")" << endl;
                                }
                                add_split(splits, Split(
                                              abstract_state_id, var, init_value,
                                              {state_value}, 1), true);
                            }
                        }
                    }
                }
            } else {
                vector<int> other_values{};
                for (int value = 0; value < domain_sizes[var]; value++) {
                    if (value != init_value && abstract_state.contains(var, value)) {
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
    Split split = split_selector.pick_split(abstract_state, move(splits), rng);
    pick_split_timer.stop();
    return utils::make_unique_ptr<Split>(move(split));
}
}
