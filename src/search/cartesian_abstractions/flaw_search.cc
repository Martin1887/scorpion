#include "flaw_search.h"

#include "abstraction.h"
#include "abstract_state.h"
#include "flaw.h"
#include "shortest_paths.h"
#include "split_selector.h"
#include "transition_system.h"
#include "utils.h"

#include "../plugins/plugin.h"
#include "../task_utils/successor_generator.h"
#include "../task_utils/task_properties.h"
#include "../utils/countdown_timer.h"
#include "../utils/memory.h"
#include "../utils/rng.h"

using namespace std;

namespace cartesian_abstractions {
int FlawSearch::get_abstract_state_id(const State &state) const {
    return abstraction.get_abstract_state_id(state);
}

Cost FlawSearch::get_h_value(int abstract_state_id) const {
    return shortest_paths.get_64bit_goal_distance(abstract_state_id);
}

OptimalTransitions FlawSearch::get_f_optimal_transitions(int abstract_state_id) const {
    return shortest_paths.get_optimal_transitions(abstraction, abstract_state_id);
}

OptimalTransitions FlawSearch::get_f_optimal_backward_transitions(int abstract_state_id) const {
    return shortest_paths.get_optimal_backward_transitions(abstraction, abstract_state_id);
}

void FlawSearch::add_flaw(int abs_id, const State &state) {
    assert(abstraction.get_state(abs_id).includes(state));

    if (log.is_at_least_debug()) {
        log << "Add flaw abs:" << abs_id << " conc:" << state.get_id() << endl;
    }

    // We limit the number of concrete states we consider per abstract state.
    // For a new abstract state (with a potentially unseen h-value),
    // this if-statement is never true.
    if (flawed_states.num_concrete_states(abs_id) >=
        max_concrete_states_per_abstract_state) {
        return;
    }

    Cost h = get_h_value(abs_id);
    if (pick_flawed_abstract_state == PickFlawedAbstractState::MIN_H) {
        if (best_flaw_h > h) {
            flawed_states.clear();
        }
        if (best_flaw_h >= h) {
            best_flaw_h = h;
            flawed_states.add_state(abs_id, state, h);
        }
    } else if (pick_flawed_abstract_state == PickFlawedAbstractState::MAX_H) {
        if (best_flaw_h < h) {
            flawed_states.clear();
        }
        if (best_flaw_h <= h) {
            best_flaw_h = h;
            flawed_states.add_state(abs_id, state, h);
        }
    } else {
        assert(pick_flawed_abstract_state == PickFlawedAbstractState::RANDOM
               || pick_flawed_abstract_state == PickFlawedAbstractState::FIRST
               || pick_flawed_abstract_state == PickFlawedAbstractState::BATCH_MIN_H);
        flawed_states.add_state(abs_id, state, h);
    }
}

void FlawSearch::initialize() {
    ++num_searches;
    last_refined_flawed_state = FlawedState::no_state;
    best_flaw_h = (pick_flawed_abstract_state == PickFlawedAbstractState::MAX_H) ? 0 : INF_COSTS;
    assert(open_list.empty());
    assert(flawed_states.empty());
    state_registry = make_unique<StateRegistry>(task_proxy);
    search_space = make_unique<SearchSpace>(*state_registry, silent_log);
    const State &initial_state = state_registry->get_initial_state();
    SearchNode node = search_space->get_node(initial_state);
    node.open_initial();
    cached_abstract_state_ids = make_unique<PerStateInformation<int>>(MISSING);
    (*cached_abstract_state_ids)[initial_state] = abstraction.get_initial_state().get_id();
    open_list.push(initial_state.get_id());
}

SearchStatus FlawSearch::step() {
    if (open_list.empty()) {
        // Completely explored f-optimal state space.
        return FAILED;
    }
    StateID id = open_list.top();
    open_list.pop();
    State s = state_registry->lookup_state(id);
    SearchNode node = search_space->get_node(s);
    assert(!node.is_closed());
    node.close();
    assert(!node.is_dead_end());
    ++num_overall_expanded_concrete_states;

    if (task_properties::is_goal_state(task_proxy, s) &&
        pick_flawed_abstract_state != PickFlawedAbstractState::MAX_H) {
        return SOLVED;
    }

    bool found_flaw = false;
    int abs_id = (*cached_abstract_state_ids)[s];
    assert(abs_id == get_abstract_state_id(s));

    // Check for each transition if the operator is applicable or if there is a deviation.
    for (auto &pair : get_f_optimal_transitions(abs_id)) {
        if (!utils::extra_memory_padding_is_reserved()) {
            return TIMEOUT;
        }

        int op_id = pair.first;
        const vector<int> &targets = pair.second;

        OperatorProxy op = task_proxy.get_operators()[op_id];

        if (!task_properties::is_applicable(op, s)) {
            // Applicability flaw
            if (!found_flaw) {
                add_flaw(abs_id, s);
                found_flaw = true;
            }
            if (pick_flawed_abstract_state == PickFlawedAbstractState::FIRST) {
                return FAILED;
            }
            continue;
        }

        State succ_state = state_registry->get_successor_state(s, op);
        SearchNode succ_node = search_space->get_node(succ_state);
        assert(!succ_node.is_dead_end());

        for (int target : targets) {
            if (!abstraction.get_state(target).includes(succ_state)) {
                // Deviation flaw
                if (!found_flaw) {
                    add_flaw(abs_id, s);
                    found_flaw = true;
                    if (log.is_at_least_debug()) {
                        s.unpack();
                        succ_state.unpack();
                        log << "Deviation flaw in transition " << endl << s.get_unpacked_values()
                            << endl << "->" << endl << succ_state.get_unpacked_values() << endl;
                        log << abstraction.get_state(abs_id) << endl << "->" << endl
                            << abstraction.get_state(target) << endl;
                        log << "Op: " << op.get_id() << ", " << op.get_name() << endl;
                        log << "Pre:" << endl;
                        for (auto pre : op.get_preconditions()) {
                            log << pre.get_pair().var << "=" << pre.get_pair().value << ", ";
                        }
                        log << endl << "Effects:";
                        for (auto eff : op.get_effects()) {
                            log << endl << "Effect: " << eff.get_fact().get_pair().var << "=" << eff.get_fact().get_pair().value << ", " << endl;
                            log << "Conds: ";
                            for (auto cond : eff.get_conditions()) {
                                log << cond.get_pair().var << "=" << cond.get_pair().value << ", ";
                            }
                        }
                        log << endl;
                    }
                }
                if (pick_flawed_abstract_state == PickFlawedAbstractState::FIRST) {
                    return FAILED;
                }
            } else if (succ_node.is_new()) {
                // No flaw
                (*cached_abstract_state_ids)[succ_state] = target;
                succ_node.open_new_node(node, op, op.get_cost());
                open_list.push(succ_state.get_id());

                if (pick_flawed_abstract_state == PickFlawedAbstractState::FIRST) {
                    // Only consider one successor.
                    break;
                }
            }
        }
        if (pick_flawed_abstract_state == PickFlawedAbstractState::FIRST) {
            // Only consider one successor as in the legacy variant.
            break;
        }
    }
    return IN_PROGRESS;
}

static void add_split(vector<vector<Split>> &splits, Split &&new_split) {
    vector<Split> &var_splits = splits[new_split.var_id];
    bool is_duplicate = false;
    for (auto &old_split : var_splits) {
        if (old_split == new_split) {
            is_duplicate = true;
            old_split.count += new_split.count;
            break;
        }
    }
    if (!is_duplicate) {
        var_splits.push_back(move(new_split));
    }
}

static void add_backward_split(vector<vector<Split>> &splits, Split &&new_split) {
    splits.push_back({move(new_split)});
}

static void add_sequence_split(vector<vector<Split>> &splits, Split &&new_split) {
    // Splits cannot be combined because they are in different abstract states.
    splits.push_back({move(new_split)});
}

static void update_affected_variables(
    const OperatorProxy &op,
    int num_variables,
    const AbstractState &state,
    vector<bool> &affected_vars,
    vector<bool> &conditionally_affected_vars,
    const Abstraction &abstraction) {
    affected_vars.assign(num_variables, false);
    conditionally_affected_vars.assign(num_variables, false);
    const vector<FactPair> &uncond_effects = abstraction.get_unconditional_effects(op.get_id());
    for (const FactPair &eff : uncond_effects) {
        affected_vars[eff.var] = true;
    }
    const vector<CondEffect> &cond_effects = abstraction.get_conditional_effects(op.get_id());
    for (const CondEffect &cond_effect : cond_effects) {
        // Conditional effects with conditions satisfied by some states
        // of the abstract state and not satisfied by other are unaffected
        // (their value depends on the concrete value).
        bool all_conds_always_satisfied = true;
        for (const FactPair &cond : cond_effect.conds) {
            if (!state.contains(cond.var, cond.value) ||
                state.count(cond.var) > 1) {
                all_conds_always_satisfied = false;
                break;
            }
        }
        if (all_conds_always_satisfied) {
            // The effect is always triggered.
            // Effects never triggered in a variable with a single value in the
            // abstract state cannot be set as affected because other effects
            // in the same variable but other values can exist.
            affected_vars[cond_effect.effect.var] = true;
        } else {
            conditionally_affected_vars[cond_effect.effect.var] = true;
        }
    }
    const vector<FactPair> &pre = abstraction.get_preconditions(op.get_id());
    for (const FactPair &precondition : pre) {
        if (!conditionally_affected_vars[precondition.var]) {
            affected_vars[precondition.var] = true;
        }
    }
}

struct FactPairHash {
    size_t operator()(FactPair fact) const {
        utils::HashState hash_state;
        hash_state.feed(fact.var);
        hash_state.feed(fact.value);
        return hash_state.get_hash64();
    }
};

static void get_deviation_splits(
    const AbstractState &abs_state,
    const vector<State> &conc_states,
    const vector<bool> &affected_variables,
    const OperatorProxy &op,
    const AbstractState &target_abs_state,
    const vector<int> &domain_sizes,
    vector<vector<Deviation>> &fact_count,
    vector<vector<EffectProxy>> &effects_in_unaffected_vars,
    vector<vector<Split>> &splits) {
    /*
      For each fact in the concrete state that is not contained in the
      target abstract state, loop over all values in the domain of the
      corresponding variable. The values that are in both the current and
      the target abstract state are the "wanted" ones, i.e., the ones that
      we want to split off. This test can be specialized for applicability and
      deviation flaws. Here, we consider deviation flaws.

      Let the desired abstract transition be (a, o, t) and the deviation be
      (a, o, b). We distinguish three cases for each variable v:

      pre(o)[v] defined: no split possible since o is applicable in s.
      pre(o)[v] undefined, eff(o)[v] defined: no split possible since regression adds whole domain.
      pre(o)[v] and eff(o)[v] undefined: if s[v] \notin t[v], wanted = intersect(a[v], b[v]).
    */
    int n_vars = domain_sizes.size();
    for (int i = 0; i < n_vars; i++) {
        for (int j = 0; j < domain_sizes[i]; j++) {
            fact_count[i][j].clear();
        }
        effects_in_unaffected_vars[i].clear();
    }
    for (auto eff : op.get_effects()) {
        int var = eff.get_fact().get_variable().get_id();
        if (!affected_variables[var]) {
            effects_in_unaffected_vars[var].push_back(eff);
        }
    }
    for (const State &conc_state : conc_states) {
        for (int var = 0; var < n_vars; var++) {
            if (!affected_variables[var]) {
                int state_value = conc_state[var].get_value();
                // With conditional effects, the source of the deviation can be
                // the condition variable or the effect variable, or both. But
                // if the abstract state has only a single value in the variable for
                // some of them, such a variable cannot be the cause (but the
                // conditions variables).
                if (abs_state.count(var) > 1) {
                    ++fact_count[var][state_value].direct_count;
                }
                for (auto eff : effects_in_unaffected_vars[var]) {
                    bool target_contains_state_value = target_abs_state.contains(var, state_value);
                    bool target_contains_effect_value = target_abs_state.contains(var, eff.get_fact().get_value());
                    // The deviation exists if the target abstract state
                    // does not contain the state value or the effect value.
                    if (!target_contains_state_value || !target_contains_effect_value) {
                        // The deviation can be fixed by forcing the condition
                        // value that makes the variable to be in the
                        // target abs state, and the wanted values are the ones
                        // that make to satisfy or not satisfy the conditions if
                        // the effect should be or not triggered respectively.
                        bool must_be_triggered = target_contains_effect_value;
                        for (auto cond : eff.get_conditions()) {
                            const FactPair &cond_pair = cond.get_pair();
                            int cond_state_value = conc_state[cond_pair.var].get_value();
                            if (must_be_triggered) {
                                if (cond_state_value != cond_pair.value &&
                                    abs_state.contains(cond_pair.var, cond_pair.value)) {
                                    fact_count[cond_pair.var][cond_state_value].cond_effect_count++;
                                    fact_count[cond_pair.var][cond_state_value].cond_effect_wanted[cond_pair.value] = true;
                                }
                            } else if (cond_state_value == cond_pair.value &&
                                       abs_state.count(cond_pair.var) > 1) {
                                fact_count[cond_pair.var][cond_state_value].cond_effect_count++;
                                // The wanted values are all other values in the abstract state.
                                for (int value = 0; value < domain_sizes[cond_pair.var]; ++value) {
                                    if (value != cond_state_value && abs_state.contains(cond_pair.var, value)) {
                                        fact_count[cond_pair.var][cond_state_value].cond_effect_wanted[value] = true;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    for (size_t var = 0; var < domain_sizes.size(); ++var) {
        for (int value = 0; value < domain_sizes[var]; ++value) {
            // Direct deviations in unaffected variables.
            if (fact_count[var][value].direct_count && !target_abs_state.contains(var, value)) {
                // Note: we could precompute the "wanted" vector, but not the split.
                vector<int> wanted;
                for (int value = 0; value < domain_sizes[var]; ++value) {
                    if (abs_state.contains(var, value) &&
                        target_abs_state.contains(var, value)) {
                        wanted.push_back(value);
                    }
                }
                // wanted may be empty if the deviation is caused by conditional effects.
                if (!wanted.empty()) {
                    add_split(splits, Split(
                                  abs_state.get_id(), var, value, move(wanted),
                                  fact_count[var][value].direct_count));
                }
            }
            // Deviations caused by non-satisfied effects conditions.
            if (fact_count[var][value].cond_effect_count) {
                assert(!fact_count[var][value].cond_effect_wanted.empty());
                vector<int> wanted;
                for (int cond_value = 0; cond_value < domain_sizes[var]; ++cond_value) {
                    if (fact_count[var][value].cond_effect_wanted[cond_value]) {
                        wanted.push_back(cond_value);
                    }
                }
                add_split(splits, Split(
                              abs_state.get_id(), var, value,
                              move(wanted),
                              fact_count[var][value].cond_effect_count));
            }
        }
    }
}

static void get_deviation_splits(
    const AbstractState &abs_state,
    const AbstractState &flaw_search_state,
    const vector<bool> &affected_variables,
    const OperatorProxy &op,
    const AbstractState &target_abs_state,
    const vector<int> &domain_sizes,
    vector<vector<Deviation>> &fact_count,
    vector<vector<EffectProxy>> &effects_in_unaffected_vars,
    vector<vector<Split>> &splits) {
    /*
      For each fact in the concrete state that is not contained in the
      target abstract state, loop over all values in the domain of the
      corresponding variable. The values that are in both the current and
      the target abstract state are the "wanted" ones, i.e., the ones that
      we want to split off. This test can be specialized for applicability and
      deviation flaws. Here, we consider deviation flaws.

      Let the desired abstract transition be (a, o, t) and the deviation be
      (a, o, b). We distinguish three cases for each variable v:

      pre(o)[v] defined: no split possible since o is applicable in s.
      pre(o)[v] undefined, eff(o)[v] defined: no split possible since regression adds whole domain.
      pre(o)[v] and eff(o)[v] undefined: if s[v] \notin t[v], wanted = intersect(a[v], b[v]).
    */
    int n_vars = domain_sizes.size();
    for (int i = 0; i < n_vars; i++) {
        for (int j = 0; j < domain_sizes[i]; j++) {
            fact_count[i][j].clear();
        }
        effects_in_unaffected_vars[i].clear();
    }
    for (auto eff : op.get_effects()) {
        int var = eff.get_fact().get_variable().get_id();
        if (!affected_variables[var]) {
            effects_in_unaffected_vars[var].push_back(eff);
        }
    }
    for (int var = 0; var < n_vars; var++) {
        if (!affected_variables[var]) {
            bool multiple_values_in_var_in_abs_state = abs_state.count(var) > 1;
            for (int state_value = 0; state_value < domain_sizes[var]; state_value++) {
                if (abs_state.contains(var, state_value) &&
                    flaw_search_state.contains(var, state_value)) {
                    // With conditional effects, the source of the deviation can be
                    // the condition variable or the effect variable, or both. But
                    // if the abstract state has only a single value in the variable for
                    // some of them, such a variable cannot be the cause (but the
                    // conditions variables).
                    if (multiple_values_in_var_in_abs_state) {
                        ++fact_count[var][state_value].direct_count;
                    }
                    for (auto eff : effects_in_unaffected_vars[var]) {
                        bool target_contains_state_value = target_abs_state.contains(var, state_value);
                        bool target_contains_effect_value = target_abs_state.contains(var, eff.get_fact().get_value());
                        // The deviation exists if the target abstract state
                        // does not contain the state value or the effect value.
                        if (!target_contains_state_value || !target_contains_effect_value) {
                            // The deviation can be fixed by forcing the condition
                            // value that makes the variable to be in the
                            // target abs state, and the wanted values are the ones
                            // that make to satisfy or not satisfy the conditions if
                            // the effect should be or not triggered respectively.
                            bool must_be_triggered = target_contains_effect_value;
                            for (auto cond : eff.get_conditions()) {
                                const FactPair &cond_pair = cond.get_pair();
                                for (int cond_state_value = 0; cond_state_value < domain_sizes[cond_pair.var]; cond_state_value++) {
                                    if (abs_state.contains(cond_pair.var, cond_state_value) &&
                                        flaw_search_state.contains(cond_pair.var, cond_state_value)) {
                                        if (must_be_triggered) {
                                            if (cond_state_value != cond_pair.value &&
                                                abs_state.contains(cond_pair.var, cond_pair.value)) {
                                                fact_count[cond_pair.var][cond_state_value].cond_effect_count++;
                                                fact_count[cond_pair.var][cond_state_value].cond_effect_wanted[cond_pair.value] = true;
                                            }
                                        } else if (cond_state_value == cond_pair.value &&
                                                   abs_state.count(cond_pair.var) > 1) {
                                            fact_count[cond_pair.var][cond_state_value].cond_effect_count++;
                                            // The wanted values are all other values in the abstract state.
                                            for (int value = 0; value < domain_sizes[cond_pair.var]; ++value) {
                                                if (value != cond_state_value && abs_state.contains(cond_pair.var, value)) {
                                                    fact_count[cond_pair.var][cond_state_value].cond_effect_wanted[value] = true;
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    for (size_t var = 0; var < domain_sizes.size(); ++var) {
        for (int value = 0; value < domain_sizes[var]; ++value) {
            // Direct deviations in unaffected variables.
            if (fact_count[var][value].direct_count && !target_abs_state.contains(var, value)) {
                // Note: we could precompute the "wanted" vector, but not the split.
                vector<int> wanted;
                for (int value = 0; value < domain_sizes[var]; ++value) {
                    if (abs_state.contains(var, value) &&
                        target_abs_state.contains(var, value)) {
                        wanted.push_back(value);
                    }
                }
                // wanted may be empty if the deviation is caused by conditional effects.
                if (!wanted.empty()) {
                    add_split(splits, Split(
                                  abs_state.get_id(), var, value, move(wanted),
                                  fact_count[var][value].direct_count));
                }
            }
            // Deviations caused by non-satisfied effects conditions.
            if (fact_count[var][value].cond_effect_count) {
                assert(!fact_count[var][value].cond_effect_wanted.empty());
                vector<int> wanted;
                for (int cond_value = 0; cond_value < domain_sizes[var]; ++cond_value) {
                    if (fact_count[var][value].cond_effect_wanted[cond_value]) {
                        wanted.push_back(cond_value);
                    }
                }
                assert(!wanted.empty());
                add_split(splits, Split(
                              abs_state.get_id(), var, value,
                              move(wanted),
                              fact_count[var][value].cond_effect_count));
            }
        }
    }
}

static void get_backward_deviation_splits(
    const AbstractState &abs_state,
    const AbstractState &flaw_search_state,
    const vector<bool> &affected_variables,
    const AbstractState &source_abs_state,
    const vector<int> &domain_sizes,
    vector<vector<bool>> &fact_count,
    vector<vector<Split>> &splits) {
    int n_vars = domain_sizes.size();
    for (int i = 0; i < n_vars; i++) {
        fact_count[i].assign(domain_sizes[i], false);
    }
    for (int var = 0; var < n_vars; var++) {
        if (!affected_variables[var]) {
            bool abs_state_has_multiple_values_in_var = abs_state.count(var) > 1;
            for (int state_value = 0; state_value < domain_sizes[var]; state_value++) {
                if (abs_state.contains(var, state_value) &&
                    flaw_search_state.contains(var, state_value)) {
                    // For conditional effects, the deviation is not always real
                    // because conditions could be satisfied in the source state,
                    // if the abstract state has only a single value in the variable
                    // for some of them, such a variable cannot be the cause.
                    if (abs_state_has_multiple_values_in_var) {
                        fact_count[var][state_value] = true;
                    }
                }
            }
        }
    }
    for (size_t var = 0; var < domain_sizes.size(); ++var) {
        for (int value = 0; value < domain_sizes[var]; ++value) {
            // Direct deviations in unaffected variables.
            if (fact_count[var][value] && !source_abs_state.contains(var, value)) {
                // Note: we could precompute the "wanted" vector, but not the split.
                vector<int> wanted;
                for (int value = 0; value < domain_sizes[var]; ++value) {
                    if (abs_state.contains(var, value) &&
                        source_abs_state.contains(var, value)) {
                        wanted.push_back(value);
                    }
                }
                assert(!wanted.empty());
                add_backward_split(splits, Split(
                                       abs_state.get_id(), var, value, move(wanted),
                                       1));
            }
        }
    }
}

// TODO: Add comment about split considering multiple transitions.
unique_ptr<Split> FlawSearch::create_split(
    const vector<StateID> &state_ids, int abstract_state_id) {
    compute_splits_timer.resume();
    const AbstractState &abstract_state = abstraction.get_state(abstract_state_id);

    if (log.is_at_least_debug()) {
        log << endl;
        log << "Create split for abstract state " << abstract_state_id << " and "
            << state_ids.size() << " concrete states." << endl;
    }

    vector<vector<Split>> splits(task_proxy.get_variables().size());
    for (auto &pair : get_f_optimal_transitions(abstract_state_id)) {
        int op_id = pair.first;
        const vector<int> &targets = pair.second;
        OperatorProxy op = task_proxy.get_operators()[op_id];

        vector<State> states;
        states.reserve(state_ids.size());
        for (const StateID &state_id : state_ids) {
            states.push_back(state_registry->lookup_state(state_id));
            assert(abstract_state.includes(states.back()));
        }

        vector<bool> applicable(states.size(), true);
        for (FactPair fact : abstraction.get_preconditions(op_id)) {
            vector<int> state_value_count(domain_sizes[fact.var], 0);
            for (size_t i = 0; i < states.size(); ++i) {
                const State &state = states[i];
                int state_value = state[fact.var].get_value();
                if (state_value != fact.value) {
                    // Applicability flaw
                    applicable[i] = false;
                    ++state_value_count[state_value];
                }
            }
            for (int value = 0; value < domain_sizes[fact.var]; ++value) {
                if (state_value_count[value] > 0) {
                    assert(value != fact.value);
                    add_split(splits, Split(
                                  abstract_state_id, fact.var, value,
                                  {fact.value}, state_value_count[value]));
                }
            }
        }

        phmap::flat_hash_map<int, vector<State>> deviation_states_by_target;
        for (size_t i = 0; i < states.size(); ++i) {
            if (!applicable[i]) {
                continue;
            }
            const State &state = states[i];
            assert(task_properties::is_applicable(op, state));
            State succ_state = state_registry->get_successor_state(state, op);
            bool target_hit = false;
            for (int target : targets) {
                if (!utils::extra_memory_padding_is_reserved()) {
                    return nullptr;
                }

                // At most one of the f-optimal targets can include the successor state.
                if (!target_hit && abstraction.get_state(target).includes(succ_state)) {
                    // No flaw
                    target_hit = true;
                } else {
                    // Deviation flaw
                    assert(target != get_abstract_state_id(succ_state));
                    deviation_states_by_target[target].push_back(state);
                }
            }
        }

        for (auto &pair : deviation_states_by_target) {
            int target = pair.first;
            const vector<State> &deviation_states = pair.second;
            if (!deviation_states.empty()) {
                int num_vars = domain_sizes.size();
                update_affected_variables(op, num_vars, abstract_state, affected_vars, conditionally_affected_vars, abstraction);
                get_deviation_splits(
                    abstract_state, deviation_states,
                    affected_vars,
                    op,
                    abstraction.get_state(target), domain_sizes,
                    deviation_fact_count, effects_in_unaffected_vars, splits);
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
    Split split = split_selector.pick_split(move(splits), rng);
    pick_split_timer.stop();
    return make_unique<Split>(move(split));
}

unique_ptr<Split> FlawSearch::create_split(
    const AbstractState &state, int abstract_state_id) {
    compute_splits_timer.resume();
    const AbstractState &abstract_state = abstraction.get_state(abstract_state_id);

    if (log.is_at_least_debug()) {
        log << endl;
        log << "Create split for abstract state " << abstract_state_id << " and "
            << "flaw-search state" << endl << state << endl;
    }

    vector<vector<Split>> splits(state.n_vars());
    bool applicable = true;
    for (auto &pair : get_f_optimal_transitions(abstract_state_id)) {
        applicable = true;
        int op_id = pair.first;
        const vector<int> &targets = pair.second;
        OperatorProxy op = task_proxy.get_operators()[op_id];

        for (const FactPair &fact : abstraction.get_preconditions(op_id)) {
            if (!state.contains(fact.var, fact.value)) {
                // Applicability flaw
                applicable = false;
                for (int value = 0; value < domain_sizes[fact.var]; ++value) {
                    if (state.contains(fact.var, value)) {
                        assert(value != fact.value);
                        add_split(splits, Split(
                                      abstract_state_id, fact.var, value,
                                      {fact.value}, 1));
                    }
                }
            }
        }

        if (!applicable) {
            if (log.is_at_least_debug()) {
                log << "Not applicable" << endl;
            }
            continue;
        }
        for (int target : targets) {
            if (!utils::extra_memory_padding_is_reserved()) {
                return nullptr;
            }

            // At most one of the f-optimal targets can include the successor state.
            if (!state.reach_with_op(abstraction.get_state(target), op, abstraction)) {
                // Deviation flaw
                if (log.is_at_least_debug()) {
                    log << "Deviation to " << abstraction.get_state(target)
                        << " with op " << op.get_id() << ":" << op.get_name() << endl;
                }
                int num_vars = domain_sizes.size();
                update_affected_variables(op, num_vars, abstract_state, affected_vars, conditionally_affected_vars, abstraction);
                get_deviation_splits(
                    abstract_state, state,
                    affected_vars,
                    op,
                    abstraction.get_state(target), domain_sizes,
                    deviation_fact_count, effects_in_unaffected_vars, splits);
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
    Split split = split_selector.pick_split(move(splits), rng);
    pick_split_timer.stop();
    return make_unique<Split>(move(split));
}

unique_ptr<Split> FlawSearch::create_split_from_goals(const AbstractState &state, int abstract_state_id) {
    compute_splits_timer.resume();
    if (log.is_at_least_debug()) {
        log << endl;
        log << "Create split for abstract state " << abstract_state_id << " and "
            << " flaw-search state:" << endl << state << endl;
    }

    const GoalsProxy goals = task_proxy.get_goals();
    vector<vector<Split>> splits = vector<vector<Split>>(task_proxy.get_variables().size());
    for (FactProxy goal : goals) {
        vector<int> other_values{};
        int var = goal.get_variable().get_id();
        int goal_value = goal.get_value();
        if (!state.contains(var, goal_value)) {
            if (log.is_at_least_debug()) {
                log << "add_split(var " << var << ", val " << goal_value << endl;
            }
            add_split(splits, Split(
                          abstract_state_id, var, -1,
                          {goal_value}, 1));
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
    Split split = split_selector.pick_split(move(splits), rng);
    pick_split_timer.stop();
    return make_unique<Split>(move(split));
}

unique_ptr<Split> FlawSearch::create_backward_split(AbstractState &&state, int abstract_state_id) {
    compute_splits_timer.resume();
    const AbstractState &abstract_state = abstraction.get_state(abstract_state_id);

    if (log.is_at_least_debug()) {
        log << endl;
        log << "Create split for abstract state " << abstract_state_id << " and "
            << "flaw-search state" << endl << state << endl;
    }

    vector<vector<Split>> splits;
    // Splits are grouped by variable only if split by wanted values (progression flaws).
    splits = vector<vector<Split>>();
    // Create the vectors only once to save memory allocations and set values in each iter.
    bool applicable = true;
    bool var_applicable = true;
    for (auto &pair : get_f_optimal_backward_transitions(abstract_state_id)) {
        applicable = true;
        if (log.is_at_least_debug()) {
            log << "Optimal backward transition(s): " << pair.first << ", "
                << pair.second << endl;
        }
        int op_id = pair.first;
        const vector<int> &sources = pair.second;
        OperatorProxy op = task_proxy.get_operators()[op_id];
        const vector<unordered_set<int>> &post_values = abstraction.get_postcondition_set(op_id);
        if (log.is_at_least_debug()) {
            log << "Operator: " << op.get_name() << endl;
        }

        int n_vars = domain_sizes.size();
        for (int var = 0; var < n_vars; var++) {
            var_applicable = state.is_backward_applicable(var, post_values[var]);
            if (!var_applicable) {
                applicable = false;
                for (int value = 0; value < domain_sizes[var]; ++value) {
                    if (state.contains(var, value) &&
                        abstract_state.contains(var, value)) {
                        if (log.is_at_least_debug()) {
                            log << "add_split(var " << var << ", val " << value
                                << ", state_value_count: "
                                << 1 << ")" << endl;
                        }
                        add_backward_split(splits, Split(
                                               abstract_state_id, var, -1,
                                               {value}, 1));
                    }
                }
            }
        }
        // Retrieving deviation flaws on states with inapplicable flaws work worse.
        if (!applicable) {
            if (log.is_at_least_debug()) {
                log << "Not applicable" << endl;
            }
            continue;
        }
        assert(state.is_backward_applicable(post_values));
        for (int source : sources) {
            if (!utils::extra_memory_padding_is_reserved()) {
                return nullptr;
            }

            // At most one of the f-optimal targets can include the successor state.
            if (!state.reach_backwards_with_op(abstraction.get_state(source), op, abstraction)) {
                // Deviation flaw
                if (log.is_at_least_debug()) {
                    log << "Deviation states by source, state: " << state
                        << ", source: " << source << endl;
                }
                const AbstractState &source_state = abstraction.get_state(source);
                update_affected_variables(op, n_vars, source_state, affected_vars, conditionally_affected_vars, abstraction);
                get_backward_deviation_splits(
                    abstract_state,
                    state,
                    affected_vars,
                    source_state, domain_sizes,
                    backward_deviation_fact_count, splits);
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
    Split split = split_selector.pick_split(move(splits), rng);
    pick_split_timer.stop();
    return make_unique<Split>(move(split));
}

unique_ptr<Split> FlawSearch::create_backward_split_from_init_state(AbstractState &&state, int abstract_state_id) {
    compute_splits_timer.resume();
    const AbstractState &abstract_state = abstraction.get_state(abstract_state_id);

    if (log.is_at_least_debug()) {
        log << endl;
        log << "Create split for abstract state " << abstract_state_id << " and "
            << "flaw-search state" << endl << state << endl;
    }

    const State init_state = task_proxy.get_initial_state();
    vector<vector<Split>> splits;
    // Splits are grouped by variable only if split by wanted values.
    splits = vector<vector<Split>>();
    int num_vars = (int)domain_sizes.size();
    for (int var = 0; var < num_vars; var++) {
        if (abstract_state.count(var) > 1) {
            int init_value = init_state[var].get_value();
            if (!state.contains(var, init_value)) {
                for (int fact_value = 0; fact_value < domain_sizes[var]; fact_value++) {
                    if (state.contains(var, fact_value) && abstract_state.contains(var, fact_value)) {
                        if (log.is_at_least_debug()) {
                            log << "add_split(var " << var << ", val " << fact_value
                                << "!=" << init_value << ")" << endl;
                        }
                        add_backward_split(splits, Split(
                                               abstract_state_id, var, init_value,
                                               {fact_value}, 1));
                    }
                }
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
    Split split = split_selector.pick_split(move(splits), rng);
    pick_split_timer.stop();
    return make_unique<Split>(move(split));
}

SearchStatus FlawSearch::search_for_flaws(const utils::CountdownTimer &cegar_timer) {
    flaw_search_timer.resume();
    if (log.is_at_least_debug()) {
        log << "Search for flaws" << endl;
    }
    initialize();
    int num_expansions_in_prev_searches = num_overall_expanded_concrete_states;
    SearchStatus search_status = IN_PROGRESS;
    while (search_status == IN_PROGRESS) {
        if (cegar_timer.is_expired()) {
            search_status = TIMEOUT;
            break;
        }

        int current_num_expanded_states = num_overall_expanded_concrete_states -
            num_expansions_in_prev_searches;
        // To remain complete, only take the expansions limit into account once at least one flaw has been found.
        if (current_num_expanded_states >= max_state_expansions && flawed_states.num_abstract_states() > 0) {
            log << "Expansion limit reached with flaws." << endl;
            search_status = FAILED;
            break;
        }
        search_status = step();
    }
    // Clear open list.
    stack<StateID>().swap(open_list);

    int current_num_expanded_states = num_overall_expanded_concrete_states -
        num_expansions_in_prev_searches;
    max_expanded_concrete_states = max(max_expanded_concrete_states,
                                       current_num_expanded_states);
    if (log.is_at_least_debug()) {
        log << "Flaw search expanded " << current_num_expanded_states
            << " states." << endl;
    }

    /* For MAX_H, we don't return SOLVED when hitting a goal state. So if MAX_H
       fails to find a single flaw, we adapt the search status here. */
    if (pick_flawed_abstract_state == PickFlawedAbstractState::MAX_H
        && search_status == FAILED && flawed_states.num_abstract_states() == 0) {
        search_status = SOLVED;
    }

    flaw_search_timer.stop();
    return search_status;
}

unique_ptr<Split> FlawSearch::get_single_split(const utils::CountdownTimer &cegar_timer) {
    auto search_status = search_for_flaws(cegar_timer);

    if (search_status == TIMEOUT)
        return nullptr;

    if (search_status == FAILED) {
        assert(!flawed_states.empty());

        FlawedState flawed_state = flawed_states.pop_random_flawed_state_and_clear(rng);
        StateID state_id = *rng.choose(flawed_state.concrete_states);

        if (log.is_at_least_debug()) {
            vector<OperatorID> trace;
            search_space->trace_path(state_registry->lookup_state(state_id), trace);
            vector<string> operator_names;
            operator_names.reserve(trace.size());
            for (OperatorID op_id : trace) {
                operator_names.push_back(task_proxy.get_operators()[op_id].get_name());
            }
            log << "Path (without last operator): " << operator_names << endl;
        }

        return create_split({state_id}, flawed_state.abs_id);
    }
    assert(search_status == SOLVED);
    return nullptr;
}

FlawedState FlawSearch::get_flawed_state_with_min_h() {
    while (!flawed_states.empty()) {
        FlawedState flawed_state = flawed_states.pop_flawed_state_with_min_h();
        Cost old_h = flawed_state.h;
        int abs_id = flawed_state.abs_id;
        assert(get_h_value(abs_id) >= old_h);
        if (get_h_value(abs_id) == old_h) {
            if (log.is_at_least_debug()) {
                log << "Reuse flawed state: " << abs_id << endl;
            }
            return flawed_state;
        } else {
            if (log.is_at_least_debug()) {
                log << "Ignore flawed state with increased f value: " << abs_id << endl;
            }
        }
    }
    // The f value increased for all states.
    return FlawedState::no_state;
}

unique_ptr<Split>
FlawSearch::get_min_h_batch_split(const utils::CountdownTimer &cegar_timer) {
    assert(pick_flawed_abstract_state == PickFlawedAbstractState::BATCH_MIN_H);
    if (last_refined_flawed_state != FlawedState::no_state) {
        // Recycle flaws of the last refined abstract state.
        Cost old_h = last_refined_flawed_state.h;
        for (const StateID &state_id : last_refined_flawed_state.concrete_states) {
            State state = state_registry->lookup_state(state_id);
            // We only add non-goal states to flawed_states.
            assert(!task_properties::is_goal_state(task_proxy, state));
            int abs_id = get_abstract_state_id(state);
            if (get_h_value(abs_id) == old_h) {
                add_flaw(abs_id, state);
            }
        }
    }

    FlawedState flawed_state = get_flawed_state_with_min_h();
    auto search_status = SearchStatus::FAILED;
    if (flawed_state == FlawedState::no_state) {
        search_status = search_for_flaws(cegar_timer);
        if (search_status == SearchStatus::FAILED) {
            flawed_state = get_flawed_state_with_min_h();
        }
    }

    if (search_status == TIMEOUT)
        return nullptr;

    if (search_status == FAILED) {
        // There are flaws to refine.
        assert(flawed_state != FlawedState::no_state);

        if (log.is_at_least_debug()) {
            log << "Use flawed state: " << flawed_state << endl;
        }

        unique_ptr<Split> split;
        split = create_split(flawed_state.concrete_states, flawed_state.abs_id);

        if (!utils::extra_memory_padding_is_reserved()) {
            return nullptr;
        }

        if (split) {
            last_refined_flawed_state = move(flawed_state);
        } else {
            last_refined_flawed_state = FlawedState::no_state;
            // We selected an abstract state without any flaws, so we try again.
            return get_min_h_batch_split(cegar_timer);
        }

        return split;
    }

    assert(search_status == SOLVED);
    return nullptr;
}

FlawSearch::FlawSearch(
    const shared_ptr<AbstractTask> &task,
    const Abstraction &abstraction,
    const ShortestPaths &shortest_paths,
    utils::RandomNumberGenerator &rng,
    PickFlawedAbstractState pick_flawed_abstract_state,
    PickSequenceFlaw pick_sequence_flaw,
    PickSplit pick_split,
    PickSplit tiebreak_split,
    bool intersect_bw_flaw_search_states,
    bool bw_progression_flaw_fallback,
    bool cache_splits,
    int max_concrete_states_per_abstract_state,
    int max_state_expansions,
    const utils::LogProxy &log) :
    task_proxy(*task),
    domain_sizes(get_domain_sizes(task_proxy)),
    abstraction(abstraction),
    shortest_paths(shortest_paths),
    split_selector(task, abstraction, pick_split, tiebreak_split, shortest_paths, log.is_at_least_debug()),
    rng(rng),
    pick_flawed_abstract_state(pick_flawed_abstract_state),
    pick_sequence_flaw(pick_sequence_flaw),
    intersect_bw_flaw_search_states(intersect_bw_flaw_search_states),
    bw_progression_flaw_fallback(bw_progression_flaw_fallback),
    cache_splits(cache_splits),
    max_concrete_states_per_abstract_state(max_concrete_states_per_abstract_state),
    max_state_expansions(max_state_expansions),
    log(log),
    silent_log(utils::get_silent_log()),
    last_refined_flawed_state(FlawedState::no_state),
    best_flaw_h((pick_flawed_abstract_state == PickFlawedAbstractState::MAX_H) ? 0 : INF),
    num_searches(0),
    num_overall_expanded_concrete_states(0),
    max_expanded_concrete_states(0),
    flaw_search_timer(false),
    compute_splits_timer(false),
    pick_split_timer(false) {
    deviation_fact_count = vector<vector<Deviation>>(domain_sizes.size());
    for (size_t var = 0; var < domain_sizes.size(); ++var) {
        deviation_fact_count[var].resize(domain_sizes[var], Deviation{0, 0, vector<bool>(domain_sizes[var], false)});
    }
    backward_deviation_fact_count = vector<vector<bool>>(domain_sizes.size());
    for (size_t var = 0; var < domain_sizes.size(); ++var) {
        backward_deviation_fact_count[var].resize(domain_sizes[var], false);
    }
    effects_in_unaffected_vars = vector<vector<EffectProxy>>(domain_sizes.size(), vector<EffectProxy>{});
    for (size_t var = 0; var < domain_sizes.size(); ++var) {
        effects_in_unaffected_vars[var].reserve(10);
    }
    affected_vars = vector<bool>(domain_sizes.size(), false);
    conditionally_affected_vars = {};
    conditionally_affected_vars.reserve(domain_sizes.size());
}

unique_ptr<Split> FlawSearch::get_split(const utils::CountdownTimer &cegar_timer) {
    unique_ptr<Split> split;

    switch (pick_flawed_abstract_state) {
    case PickFlawedAbstractState::FIRST:
    case PickFlawedAbstractState::RANDOM:
    case PickFlawedAbstractState::MIN_H:
    case PickFlawedAbstractState::MAX_H:
        split = get_single_split(cegar_timer);
        break;
    case PickFlawedAbstractState::BATCH_MIN_H:
        split = get_min_h_batch_split(cegar_timer);
        break;
    default:
        log << "Invalid pick flaw strategy: "
            << static_cast<int>(pick_flawed_abstract_state)
            << endl;
        utils::exit_with(utils::ExitCode::SEARCH_INPUT_ERROR);
    }

    if (split) {
        assert((pick_flawed_abstract_state != PickFlawedAbstractState::MAX_H
                && pick_flawed_abstract_state != PickFlawedAbstractState::MIN_H)
               || best_flaw_h == get_h_value(split->abstract_state_id));
    }
    return split;
}

unique_ptr<Split> FlawSearch::get_split_legacy(const Solution &solution) {
    state_registry = make_unique<StateRegistry>(task_proxy);
    bool debug = log.is_at_least_debug();
    if (debug)
        log << "Check solution:" << endl;

    const AbstractState *abstract_state = &abstraction.get_initial_state();
    State concrete_state = state_registry->get_initial_state();
    assert(abstract_state->includes(concrete_state));

    if (debug)
        log << "  Initial abstract state: " << *abstract_state << endl;

    for (const Transition &step : solution) {
        OperatorProxy op = task_proxy.get_operators()[step.op_id];
        const AbstractState *next_abstract_state = &abstraction.get_state(step.target_id);
        if (task_properties::is_applicable(op, concrete_state)) {
            if (debug)
                log << "  Move to " << *next_abstract_state << " with "
                    << op.get_name() << endl;
            State next_concrete_state = state_registry->get_successor_state(concrete_state, op);
            if (!next_abstract_state->includes(next_concrete_state)) {
                if (debug)
                    log << "  Paths deviate." << endl;
                return create_split({concrete_state.get_id()}, abstract_state->get_id());
            }
            abstract_state = next_abstract_state;
            concrete_state = move(next_concrete_state);
        } else {
            if (debug)
                log << "  Operator not applicable: " << op.get_name() << endl;
            return create_split({concrete_state.get_id()}, abstract_state->get_id());
        }
    }
    assert(abstraction.get_goals().count(abstract_state->get_id()));
    if (task_properties::is_goal_state(task_proxy, concrete_state)) {
        // We found a concrete solution.
        return nullptr;
    } else {
        if (debug)
            log << "  Goal test failed." << endl;
        return create_split({concrete_state.get_id()}, abstract_state->get_id());
    }
}

unique_ptr<Split> FlawSearch::get_backward_split(const Solution &solution) {
    bool debug = log.is_at_least_debug();
    if (debug) {
        log << "Check solution:" << endl;
        for (size_t i = 0; i < solution.size(); i++) {
            log << solution.at(i) << endl;
        }
        log << "  Abstract plan:" << endl;
        for (const Transition &step : solution) {
            OperatorProxy op = task_proxy.get_operators()[step.op_id];
            log << "    " << op.get_name() << endl;
        }
    }

    const AbstractState *initial_abstract_state = &abstraction.get_initial_state();
    const AbstractState *abstract_state;
    if (solution.empty()) {
        abstract_state = initial_abstract_state;
    } else {
        abstract_state = &abstraction.get_state(solution.back().target_id);
    }

    // The concrete transition system trace starts in the goals.
    vector<FactPair> goals_facts = task_properties::get_fact_pairs(task_proxy.get_goals());
    // The goal state if intersect with abstract states and goals otherwise.
    AbstractState flaw_search_state = intersect_bw_flaw_search_states ?
        AbstractState(-1, -1, abstract_state->clone_cartesian_set()) :
        AbstractState(-1, -1, get_domain_sizes(task_proxy), move(goals_facts), true);
    if (debug) {
        log << "  Initial abstract state: " << *initial_abstract_state << endl;
        log << "  Start (goal) abstract state: " << *abstract_state << endl;
        log << "  Start (goal) flaw search state: " << flaw_search_state << endl;
    }

    // Iterate over solution in reverse direction.
    for (int i = solution.size() - 1; i >= 0; i--) {
        const Transition &step = solution.at(i);
        const OperatorProxy &op = task_proxy.get_operators()[step.op_id];
        const vector<unordered_set<int>> &post_values = abstraction.get_postcondition_set(step.op_id);
        if (flaw_search_state.is_backward_applicable(post_values)) {
            const AbstractState *next_abstract_state;
            if (i > 0) {
                next_abstract_state = &abstraction.get_state(solution.at(i - 1).target_id);
            } else {
                next_abstract_state = initial_abstract_state;
            }
            if (debug) {
                log << endl << "  Move from " << *abstract_state << " to " << *next_abstract_state << " with "
                    << op.get_name() << endl;
                log << "  In flaw-search space move from "
                    << flaw_search_state << " with " << op.get_name() << endl;
            }
            if (!flaw_search_state.reach_backwards_with_op(*next_abstract_state, op, abstraction)) {
                if (debug) {
                    log << "  Paths deviate." << endl;
                    log << "  Flaw-search state: " << flaw_search_state << endl;
                    log << "  Previous abstract state: " << *abstract_state << endl;
                    log << "  Abstract state: " << *next_abstract_state << endl;
                }
                return create_backward_split(move(flaw_search_state), abstract_state->get_id());
            } else {
                abstract_state = next_abstract_state;
                flaw_search_state.regress(op, abstraction);
                if (intersect_bw_flaw_search_states) {
                    flaw_search_state.intersect(*abstract_state);
                }
                if (debug) {
                    log << "  In flaw-search space move to "
                        << flaw_search_state << " with " << op.get_name() << endl;
                }
            }
        } else {
            if (debug)
                log << "  Operator not backward applicable: " << op.get_name() << endl;

            return create_backward_split(move(flaw_search_state), abstract_state->get_id());
        }
    }
    assert(initial_abstract_state->get_id() == abstract_state->get_id());
    if (flaw_search_state.includes(task_proxy.get_initial_state())) {
        // No flaws, search a progression flaw or handle it as a concrete
        // solution has been found (probably not because the over-approximation
        // of non-Cartesian regression).
        if (bw_progression_flaw_fallback) {
            return get_split_legacy(solution);
        } else {
            return nullptr;
        }
    } else {
        if (debug)
            log << "  Initial state test failed." << endl;

        return create_backward_split_from_init_state(move(flaw_search_state), abstract_state->get_id());
    }
}

unique_ptr<Split> FlawSearch::get_sequence_split(const Solution &solution) {
    if (!utils::extra_memory_padding_is_reserved()) {
        return nullptr;
    }
    bool debug = log.is_at_least_debug();
    if (debug)
        log << "Check solution:" << endl;

    const AbstractState *abstract_state = &abstraction.get_initial_state();

    state_registry = make_unique<StateRegistry>(task_proxy);
    AbstractState flaw_search_state =
        AbstractState(-1, -1, get_domain_sizes(task_proxy),
                      task_properties::get_fact_pairs(state_registry->get_initial_state()));
    assert(abstract_state->includes(flaw_search_state));

    // flaw-search state, abstract state, flaw in goals.
    vector<tuple<AbstractState, int, bool>> flaws{};
    flaws.reserve(solution.size());

    if (debug)
        log << "  Initial abstract state: " << *abstract_state << endl;

    for (const Transition &step : solution) {
        if (!utils::extra_memory_padding_is_reserved()) {
            return nullptr;
        }
        OperatorProxy op = task_proxy.get_operators()[step.op_id];
        const AbstractState *next_abstract_state = &abstraction.get_state(step.target_id);
        if (flaw_search_state.is_applicable(abstraction.get_preconditions(step.op_id))) {
            if (debug)
                log << "  Move to " << *next_abstract_state << " with "
                    << op.get_name() << endl;
            if (!flaw_search_state.reach_with_op(*next_abstract_state, op, abstraction)) {
                if (debug) {
                    log << "  Paths deviate." << endl;
                    log << "  Previous flaw-search state: " << flaw_search_state << endl;
                    log << "  Previous abstract state: " << *abstract_state << endl;
                }
                flaws.push_back({flaw_search_state, abstract_state->get_id(), false});
                flaw_search_state.progress(op, abstraction);
                if (debug) {
                    log << "  Flaw-search state: " << flaw_search_state << endl;
                }
                flaw_search_state.undeviate(*next_abstract_state);
                if (debug) {
                    log << "  Undeviated state: " << flaw_search_state << endl;
                    log << "  Abstract state: " << *next_abstract_state << endl;
                }
            } else {
                flaw_search_state.progress(op, abstraction);
            }
            if (debug)
                log << "  Move to " << flaw_search_state << " with "
                    << op.get_name() << endl;
            abstract_state = next_abstract_state;
        } else {
            if (debug) {
                log << "  Operator not applicable: " << op.get_name() << endl;
                log << "  Abstract state: " << *abstract_state << endl;
                log << "  Flaw-search state: " << flaw_search_state << endl;
            }
            flaws.push_back({flaw_search_state, abstract_state->get_id(), false});
            abstract_state = &abstraction.get_state(step.target_id);
            // Apply the operator as if it were applicable (and undeviate if needed).
            flaw_search_state.progress(op, abstraction);
            if (debug) {
                log << "  Move to " << *next_abstract_state << " with "
                    << op.get_name() << endl;
                log << "  Move to " << flaw_search_state << " with "
                    << op.get_name() << endl;
            }
            if (!abstract_state->is_superset_of(flaw_search_state)) {
                if (debug) {
                    log << "  The state " << flaw_search_state << " is not a subset or equal" << endl;
                    log << "  Abstract state: " << *abstract_state << endl;
                }
                flaw_search_state.undeviate(*abstract_state);
                if (debug)
                    log << "  Undeviated state: " << flaw_search_state << endl;
            }
        }
    }
    assert(abstraction.get_goals().count(abstract_state->get_id()));

    if (flaws.empty() || !utils::extra_memory_padding_is_reserved()) {
        return nullptr;
    } else {
        // Cache is useless when splits are computed only in one state
        // because it is invalidated for the split state.
        if (pick_sequence_flaw == PickSequenceFlaw::LAST_FLAW) {
            auto [flaw_search_state, abstract_state_id, in_goals] = move(flaws.back());
            if (in_goals) {
                return create_split_from_goals(move(flaw_search_state), abstract_state_id);
            } else {
                return create_split(move(flaw_search_state), abstract_state_id);
            }
        } else {
            vector<vector<Split>> splits(task_proxy.get_variables().size());
            for (auto &&[flaw_search_state, abstract_state_id, in_goals] : flaws) {
                if (!utils::extra_memory_padding_is_reserved()) {
                    return nullptr;
                }
                if (cache_splits) {
                    add_sequence_split(splits,
                                       splits_cache_get(move(flaw_search_state),
                                                        abstract_state_id,
                                                        in_goals));
                } else {
                    if (in_goals) {
                        add_sequence_split(splits, move(*create_split_from_goals(flaw_search_state, abstract_state_id)));
                    } else {
                        add_sequence_split(splits, move(*create_split(flaw_search_state, abstract_state_id)));
                    }
                }
            }
            pick_split_timer.resume();
            Split split = split_selector.pick_split(move(splits), rng);
            pick_split_timer.stop();
            // The state is split, so cache must be invalidated for it.
            if (cache_splits) {
                splits_cache_invalidate(split.abstract_state_id);
            }
            return make_unique<Split>(move(split));
        }
    }
}

void FlawSearch::print_statistics() const {
    int refinements = abstraction.get_num_states() - 1;
    int expansions = num_overall_expanded_concrete_states;
    log << "Flaw searches: " << num_searches << endl;
    log << "Expanded concrete states: " << expansions << endl;
    log << "Maximum expanded concrete states in single flaw search: "
        << max_expanded_concrete_states << endl;
    log << "Flaw search time: " << flaw_search_timer << endl;
    log << "Time for computing splits: " << compute_splits_timer << endl;
    log << "Time for selecting splits: " << pick_split_timer << endl;
    if (num_searches > 0) {
        log << "Average number of refinements per flaw search: "
            << refinements / static_cast<float>(num_searches) << endl;
        log << "Average number of expanded concrete states per flaw search: "
            << expansions / static_cast<float>(num_searches) << endl;
        log << "Average flaw search time: " << flaw_search_timer() / num_searches << endl;
    }
}

Split FlawSearch::splits_cache_get(AbstractState &&flaw_search_state,
                                   int abstract_state_id,
                                   bool split_goals) {
    tuple<AbstractState, int, bool> flaw = {move(flaw_search_state), abstract_state_id, split_goals};
    OptimalTransitions opt_tr = get_f_optimal_transitions(abstract_state_id);
    // Check split is cached and f-optimal transitions have not changed.
    if (splits_cache.count(abstract_state_id) == 0 ||
        splits_cache[abstract_state_id].count(flaw) == 0 ||
        opt_tr_cache[abstract_state_id] != opt_tr) {
        splits_cache[abstract_state_id].erase(flaw);
        if (split_goals) {
            splits_cache[abstract_state_id].emplace(flaw, create_split_from_goals(get<0>(flaw), abstract_state_id));
        } else {
            splits_cache[abstract_state_id].emplace(flaw, create_split(get<0>(flaw), abstract_state_id));
        }
        opt_tr_cache.erase(abstract_state_id);
        opt_tr_cache.emplace(abstract_state_id, std::move(opt_tr));
    }
    auto split =
        splits_cache[abstract_state_id].at(flaw);
    return Split(
        split->abstract_state_id, split->var_id, split->value,
        vector<int>(split->values), split->count);
}

void FlawSearch::splits_cache_invalidate(int abstract_state_id) {
    if (!splits_cache.empty()) {
        splits_cache.erase(abstract_state_id);
        // Invalidate cache of flaws with incoming/outgoing
        // transitions to this state. f-optimal only are not
        // enough, all transitions must be invalidated.
        // Flaws in goals are not necessary
        // to be invalidated, but detecting them is more expensive and they are
        // a low percentage of flaws.
        for (auto &&tr :
             abstraction.get_incoming_transitions(abstract_state_id)) {
            if (splits_cache.count(tr.target_id) > 0) {
                splits_cache.erase(tr.target_id);
                opt_tr_cache.erase(tr.target_id);
            }
        }
    }
}

static plugins::TypedEnumPlugin<PickFlawedAbstractState> _enum_plugin({
        {"first",
         "Consider first encountered flawed abstract state and a random concrete state."},
        {"first_on_shortest_path",
         "Follow the arbitrary solution in the shortest path tree (no flaw search). "
         "Consider first encountered flawed abstract state and a random concrete state."},
        {"first_on_shortest_path_backward",
         "First regression flaw."
         "Consider first encountered flawed abstract state and a random concrete state."},
        {"random",
         "Collect all flawed abstract states and then consider a random abstract state "
         "and a random concrete state."},
        {"min_h",
         "Collect all flawed abstract states and then consider a random abstract state "
         "with minimum h value and a random concrete state."},
        {"max_h",
         "Collect all flawed abstract states and then consider a random abstract state "
         "with maximum h value and a random concrete state."},
        {"batch_min_h",
         "Collect all flawed abstract states and iteratively refine them (by increasing "
         "h value). Only start a new flaw search once all remaining flawed abstract "
         "states are refined. For each abstract state consider all concrete states."},
        {"sequence",
         "Collect progression sequence flaws and choose one of them by pick sequence flaw"
         "and pick split."}
    });

static plugins::TypedEnumPlugin<PickSequenceFlaw> _enum_sequence_plugin({
        {"all_flaws",
         "Consider the best split among all flaws by the pick_split and tiebreaks."},
        {"last_flaw",
         "Consider the best split by pick_split only in the last flawed state."}
    });
}
