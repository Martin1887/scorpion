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
void FlawSearch::get_deviation_splits(
    const AbstractState &abs_state,
    const vector<AbstractState> &flaw_search_states,
    const vector<int> &unaffected_variables,
    const AbstractState &target_abs_state,
    const vector<int> &domain_sizes,
    const int op_cost,
    vector<vector<Split>> &splits,
    bool split_unwanted_values) {
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
    // Note: it could be faster to use an efficient hash map for this.
    vector<vector<int>> fact_count(domain_sizes.size());
    vector<bool> var_fact_count(domain_sizes.size());
    for (size_t var = 0; var < domain_sizes.size(); ++var) {
        fact_count[var].resize(domain_sizes[var], 0);
    }
    for (const AbstractState &fs_state : flaw_search_states) {
        for (int var : unaffected_variables) {
            // When disambiguation is implemented, `contains` will be possible
            // instead of `intersects`
            if (!target_abs_state.domain_subsets_intersect(fs_state, var)) {
                for (int value : fs_state.get_cartesian_set().get_values(var)) {
                    if (abs_state.contains(var, value)) {
                        ++fact_count[var][value];
                        var_fact_count[var] = true;
                    }
                }
            }
        }
    }
    for (size_t var = 0; var < domain_sizes.size(); ++var) {
        vector<int> wanted;
        if (var_fact_count[var]) {
            for (int value = 0; value < domain_sizes[var]; ++value) {
                if (abs_state.contains(var, value) &&
                    target_abs_state.contains(var, value)) {
                    wanted.push_back(value);
                }
            }
            for (int value = 0; value < domain_sizes[var]; ++value) {
                if (fact_count[var][value] && !target_abs_state.contains(var, value)) {
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

unique_ptr<Split> FlawSearch::create_split(
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

    const TransitionSystem &ts = abstraction.get_transition_system();
    vector<vector<Split>> splits;
    // Splits are grouped by variable only if split by wanted values.
    if (split_unwanted_values) {
        splits = vector<vector<Split>>();
    } else {
        splits = vector<vector<Split>>(task_proxy.get_variables().size());
    }
    for (auto &pair : get_f_optimal_transitions(abstract_state_id)) {
        int op_id = pair.first;
        const vector<int> &targets = pair.second;
        OperatorProxy op = task_proxy.get_operators()[op_id];

        vector<bool> applicable(states.size(), true);
        for (FactPair fact : ts.get_preconditions(op_id)) {
            vector<int> state_value_count(domain_sizes[fact.var], 0);
            for (size_t i = 0; i < states.size(); ++i) {
                const AbstractState &state = states[i];
                if (!state.contains(fact.var, fact.value)) {
                    // Applicability flaw
                    applicable[i] = false;
                    for (int value : state.get_cartesian_set().get_values(fact.var)) {
                        if (abstract_state.contains(fact.var, value)) {
                            ++state_value_count[value];
                        }
                    }
                }
            }
            for (int value = 0; value < domain_sizes[fact.var]; ++value) {
                if (state_value_count[value] > 0) {
                    assert(value != fact.value);
                    if (split_unwanted_values) {
                        add_split(splits, Split(
                                      abstract_state_id, fact.var, fact.value,
                                      {value}, state_value_count[value],
                                      op.get_cost()), true);
                    } else {
                        add_split(splits, Split(
                                      abstract_state_id, fact.var, value,
                                      {fact.value}, state_value_count[value],
                                      op.get_cost()));
                    }
                }
            }
        }

        phmap::flat_hash_map<int, vector<AbstractState>> deviation_states_by_target;
        for (size_t i = 0; i < states.size(); ++i) {
            if (!applicable[i]) {
                continue;
            }
            const AbstractState &state = states[i];
            assert(state.is_applicable(op));
            AbstractState succ_state(-1, -1, state.progress(op));
            bool target_hit = false;
            for (int target : targets) {
                if (!utils::extra_memory_padding_is_reserved()) {
                    return nullptr;
                }

                // At most one of the f-optimal targets can include the successor state.
                if (!target_hit && abstraction.get_state(target).intersects(succ_state)) {
                    // No flaw
                    target_hit = true;
                } else {
                    // Deviation flaw
                    deviation_states_by_target[target].push_back(state);
                }
            }
        }

        for (auto &pair : deviation_states_by_target) {
            int target = pair.first;
            const vector<AbstractState> &deviation_states = pair.second;
            if (!deviation_states.empty()) {
                int num_vars = domain_sizes.size();
                get_deviation_splits(
                    abstract_state, deviation_states,
                    get_unaffected_variables(op, num_vars),
                    abstraction.get_state(target), domain_sizes, op.get_cost(),
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

unique_ptr<Split> FlawSearch::create_split_from_goal_state(
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

    const GoalsProxy goals = task_proxy.get_goals();
    vector<vector<Split>> splits = vector<vector<Split>>(task_proxy.get_variables().size());
    int num_vars = (int)domain_sizes.size();
    for (int var = 0; var < num_vars; var++) {
        if (abstract_state.count(var) > 1) {
            for (FactProxy goal : goals) {
                vector<int> other_values{};
                int goal_value = goal.get_value();
                if (goal.get_variable().get_id() == var) {
                    for (int value = 0; value < domain_sizes[var]; value++) {
                        if (value != goal_value && abstract_state.contains(var, value)) {
                            other_values.push_back(value);
                        }
                    }

                    if (split_unwanted_values) {
                        for (AbstractState state : states) {
                            for (int value : state.get_cartesian_set().get_values(var)) {
                                if (value != goal_value && abstract_state.contains(var, value)) {
                                    if (log.is_at_least_debug()) {
                                        log << "add_split(var " << var << ", val " << value
                                            << "!=" << goal_value << ")" << endl;
                                    }
                                    add_split(splits, Split(
                                                  abstract_state_id, var, goal_value,
                                                  {value}, 1), true);
                                }
                            }
                        }
                    } else {
                        if (log.is_at_least_debug()) {
                            log << "add_split(var " << var << ", val " << goal_value
                                << "!=" << other_values << ")" << endl;
                        }
                        add_split(splits, Split(
                                      abstract_state_id, var, goal_value,
                                      move(other_values), 1));
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
    Split split = split_selector.pick_split(abstract_state, move(splits), rng);
    pick_split_timer.stop();
    return utils::make_unique_ptr<Split>(move(split));
}

vector<LegacyFlaw> FlawSearch::get_forward_flaws(const Solution &solution,
                                                 const bool in_sequence,
                                                 const InAbstractionFlawSearchKind only_in_abstraction) {
    vector<LegacyFlaw> flaws{};
    state_registry = utils::make_unique_ptr<StateRegistry>(task_proxy);
    bool debug = log.is_at_least_debug();
    if (debug)
        log << "Check solution:" << endl;

    int start_abstract_state_index = 0;
    const AbstractState *abstract_state = &abstraction.get_initial_state();
    AbstractState flaw_search_state = AbstractState(abstraction.get_initial_state());
    switch (only_in_abstraction) {
    case InAbstractionFlawSearchKind::TRUE:
        // Already correct values.
        break;
    case InAbstractionFlawSearchKind::FALSE:
        flaw_search_state = AbstractState(get_domain_sizes(task_proxy),
                                          task_properties::get_fact_pairs(state_registry->get_initial_state()));
        break;
    case InAbstractionFlawSearchKind::ITERATIVE_IN_REGRESSION:
        // In goal state and in the last state before goal no possible flaw
        // exists, so get the previous state, or the initial state if 2 or
        // fewer states.
        if (solution.size() >= 3) {
            flaw_search_state = AbstractState(abstraction.get_state(solution.at(solution.size() - 3).target_id));
            abstract_state = &abstraction.get_state(solution.at(solution.size() - 3).target_id);
            start_abstract_state_index = solution.size() - 2;
        } else if (solution.size() == 2) {
            // Already correct values.
            start_abstract_state_index = 0;
        } else {
            flaw_search_state = AbstractState(get_domain_sizes(task_proxy),
                                              task_properties::get_fact_pairs(state_registry->get_initial_state()));
            start_abstract_state_index = -1;
        }
        break;
    }
    assert(abstract_state->intersects(flaw_search_state));

    if (debug) {
        log << "  Initial abstract state: " << *abstract_state << endl;
        log << "  Abstract plan:" << endl;
        for (const Transition &step : solution) {
            OperatorProxy op = task_proxy.get_operators()[step.op_id];
            log << "    " << op.get_name() << endl;
        }
    }

    int solution_size = (int)solution.size();
    do {
        for (int i = max(start_abstract_state_index, 0); i < solution_size; i++) {
            Transition step = solution.at(i);
            OperatorProxy op = task_proxy.get_operators()[step.op_id];
            const AbstractState *next_abstract_state = &abstraction.get_state(step.target_id);
            if (flaw_search_state.is_applicable(op)) {
                if (debug)
                    log << "  Move to " << *next_abstract_state << " with "
                        << op.get_name() << endl;
                AbstractState next_flaw_search_state(-1, -1, flaw_search_state.progress(op));
                if (!next_abstract_state->intersects(next_flaw_search_state)) {
                    if (debug) {
                        log << "  Paths deviate." << endl;
                        log << "  Flaw-search state: " << next_flaw_search_state << endl;
                    }
                    flaws.push_back(LegacyFlaw(move(flaw_search_state), abstract_state->get_id(), false));
                    if (!in_sequence ||
                        split_selector.sequence_pick == PickSequenceFlaw::FIRST_FLAW) {
                        return flaws;
                    } else {
                        next_flaw_search_state = AbstractState(-1, -1, next_flaw_search_state.undeviate(*next_abstract_state));
                        if (debug) {
                            log << "  Undeviated state: " << next_flaw_search_state << endl;
                            log << "  Abstract state: " << *next_abstract_state << endl;
                        }
                    }
                }
                abstract_state = next_abstract_state;
                flaw_search_state = move(next_flaw_search_state);
            } else {
                if (debug) {
                    log << "  Operator not applicable: " << op.get_name() << endl;
                    log << "  Operator preconditions: " << endl;
                    for (FactProxy precond : op.get_preconditions()) {
                        log << "    " << precond.get_pair() << endl;
                    }
                    log << "  Abstract state: " << *abstract_state << endl;
                    log << "  Flaw-search state: " << flaw_search_state << endl;
                }
                flaws.push_back(LegacyFlaw(move(flaw_search_state), abstract_state->get_id(), false));
                if (!in_sequence ||
                    split_selector.sequence_pick == PickSequenceFlaw::FIRST_FLAW) {
                    return flaws;
                } else {
                    abstract_state = &abstraction.get_state(step.target_id);
                    // Apply the operator as if it were applicable (and undeviate if needed).
                    flaw_search_state = AbstractState(-1, -1, flaws.back().flaw_search_state.progress(op));
                    if (!abstract_state->intersects(flaw_search_state)) {
                        if (debug) {
                            log << "  The state " << flaw_search_state << " does not intersects" << endl;
                            log << "  Abstract state: " << *abstract_state << endl;
                        }
                        flaw_search_state = AbstractState(-1, -1, flaw_search_state.undeviate(*abstract_state));
                        if (debug)
                            log << "  Undeviated state: " << flaw_search_state << endl;
                    }
                }
            }
        }
        assert(abstraction.get_goals().count(abstract_state->get_id()));
        if (only_in_abstraction != InAbstractionFlawSearchKind::TRUE) {
            if (!flaw_search_state.includes(task_properties::get_fact_pairs(task_proxy.get_goals()))) {
                // This may happen if goals are not separated from the initial state
                // before getting splits (bidirectional strategies so far),
                // and it needs a special function to do it because goal state
                // has no optimal transitions.
                if (debug)
                    log << "  Goal test failed." << endl;
                flaws.push_back(LegacyFlaw(move(flaw_search_state), abstract_state->get_id(), true));
            }
        }
        if (only_in_abstraction == InAbstractionFlawSearchKind::ITERATIVE_IN_REGRESSION && flaws.empty()) {
            if (start_abstract_state_index < 0) {
                break;
            } else if (start_abstract_state_index == 0) {
                flaw_search_state = AbstractState(get_domain_sizes(task_proxy),
                                                  task_properties::get_fact_pairs(state_registry->get_initial_state()));
                abstract_state = &abstraction.get_initial_state();
            } else if (start_abstract_state_index == 1) {
                flaw_search_state = AbstractState(abstraction.get_initial_state());
                abstract_state = &abstraction.get_initial_state();
            } else {
                flaw_search_state = AbstractState(abstraction.get_state(solution.at(start_abstract_state_index - 2).target_id));
                abstract_state = &abstraction.get_state(solution.at(start_abstract_state_index - 2).target_id);
            }
        }
        start_abstract_state_index--;
    } while (only_in_abstraction == InAbstractionFlawSearchKind::ITERATIVE_IN_REGRESSION && flaws.empty());

    return flaws;
}

vector<LegacyFlaw> FlawSearch::get_backward_flaws(const Solution &solution,
                                                  const bool in_sequence,
                                                  const InAbstractionFlawSearchKind only_in_abstraction) {
    vector<LegacyFlaw> flaws{};
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

    // The concrete transition system trace starts in the goals,
    // that usually is an abstract state
    GoalsProxy goals = task_proxy.get_goals();
    vector<FactPair> goals_facts = task_properties::get_fact_pairs(task_proxy.get_goals());
    AbstractState flaw_search_state = only_in_abstraction != InAbstractionFlawSearchKind::FALSE ?
        AbstractState(-1, -1, abstract_state->get_cartesian_set())
        :
        AbstractState(get_domain_sizes(task_proxy), move(goals_facts));
    if (intersect_flaw_search_abstract_states) {
        flaw_search_state = flaw_search_state.intersection(*abstract_state);
    }
    if (debug) {
        log << "  Initial abstract state: " << *initial_abstract_state << endl;
        log << "  Start (goal) abstract state: " << *abstract_state << endl;
        log << "  Start (goal) flaw search state: " << flaw_search_state << endl;
    }

    // iterate over solution in reverse direction
    for (int i = solution.size() - 1; i >= 0; i--) {
        Transition step = solution.at(i);
        OperatorProxy op = task_proxy.get_operators()[step.op_id];
        if (flaw_search_state.is_backward_applicable(op)) {
            const AbstractState *next_abstract_state;
            if (i > 0) {
                next_abstract_state = &abstraction.get_state(solution.at(i - 1).target_id);
            } else {
                next_abstract_state = initial_abstract_state;
            }
            if (debug)
                log << "  Move from " << *abstract_state << " to " << *next_abstract_state << " with "
                    << op.get_name() << endl;
            AbstractState next_flaw_search_state(AbstractState(-1, -1, flaw_search_state.regress(op)));
            if (debug)
                log << "  In flaw-search space move from " << flaw_search_state << " to "
                    << next_flaw_search_state << " with " << op.get_name() << endl;
            if (!next_abstract_state->intersects(next_flaw_search_state)) {
                if (debug) {
                    log << "  Paths deviate." << endl;
                    log << "  Flaw-search state: " << next_flaw_search_state << endl;
                }
                flaws.push_back(LegacyFlaw(move(flaw_search_state), abstract_state->get_id(), false));
                if (!in_sequence ||
                    only_in_abstraction == InAbstractionFlawSearchKind::ITERATIVE_IN_REGRESSION ||
                    split_selector.sequence_pick == PickSequenceFlaw::FIRST_FLAW ||
                    split_selector.sequence_pick == PickSequenceFlaw::CLOSEST_TO_GOAL_FLAW) {
                    return flaws;
                } else {
                    next_flaw_search_state = AbstractState(-1, -1, next_flaw_search_state.undeviate(*next_abstract_state));
                    if (debug) {
                        log << "  Undeviated state: " << next_flaw_search_state << endl;
                    }
                }
            }
            abstract_state = next_abstract_state;
            flaw_search_state = move(next_flaw_search_state);
            if (intersect_flaw_search_abstract_states) {
                flaw_search_state = flaw_search_state.intersection(*abstract_state);
                if (debug)
                    log << "  Intersected flaw-search state: " << flaw_search_state << endl;
            }
        } else {
            if (debug)
                log << "  Operator not backward applicable: " << op.get_name() << endl;
            flaws.push_back(LegacyFlaw(move(flaw_search_state), abstract_state->get_id(), false));
            if (!in_sequence ||
                only_in_abstraction == InAbstractionFlawSearchKind::ITERATIVE_IN_REGRESSION ||
                split_selector.sequence_pick == PickSequenceFlaw::FIRST_FLAW ||
                split_selector.sequence_pick == PickSequenceFlaw::CLOSEST_TO_GOAL_FLAW) {
                return flaws;
            } else {
                if (i > 0) {
                    abstract_state = &abstraction.get_state(solution.at(i - 1).target_id);
                } else {
                    abstract_state = initial_abstract_state;
                }
                // Apply the operator as if it were applicable (and undeviate if needed).
                flaw_search_state = AbstractState(-1, -1, flaws.back().flaw_search_state.regress(op));
                if (!abstract_state->intersects(flaw_search_state)) {
                    if (debug) {
                        log << "  The state " << flaw_search_state << " does not intersects" << endl;
                        log << "  Abstract state: " << *abstract_state << endl;
                    }
                    flaw_search_state = AbstractState(-1, -1, flaw_search_state.undeviate(*abstract_state));
                    if (debug)
                        log << "  Undeviated state: " << flaw_search_state << endl;
                }
            }
        }
    }
    assert(initial_abstract_state->get_id() == abstract_state->get_id());
    if (only_in_abstraction != InAbstractionFlawSearchKind::TRUE) {
        if (!flaw_search_state.includes(task_proxy.get_initial_state())) {
            // This only happens if the initial abstract state is not refined
            // before starting the refinement steps.
            if (debug)
                log << "  Initial state test failed." << endl;
            flaws.push_back(LegacyFlaw(move(flaw_search_state), abstract_state->get_id(), true));
        }
    }

    // It could happen that flaws are not found because starting in the goals
    // abstract state, so start from the goals.
    if (only_in_abstraction == InAbstractionFlawSearchKind::ITERATIVE_IN_REGRESSION && flaws.empty()) {
        flaws = get_backward_flaws(solution, true, InAbstractionFlawSearchKind::FALSE);
    }

    return flaws;
}

SplitProperties FlawSearch::get_sequence_splits(const Solution &solution,
                                                const InAbstractionFlawSearchKind only_in_abstraction,
                                                const bool forward,
                                                const bool backward) {
    assert(forward || backward);
    vector<LegacyFlaw> forward_flaws{};
    vector<LegacyFlaw> backward_flaws{};
    if (forward) {
        forward_flaws = get_forward_flaws(solution, true, only_in_abstraction);
        if (only_in_abstraction == InAbstractionFlawSearchKind::TRUE && forward_flaws.empty()) {
            forward_flaws = get_forward_flaws(solution, true, InAbstractionFlawSearchKind::FALSE);
        }
    }
    if (backward) {
        backward_flaws = get_backward_flaws(solution, true, only_in_abstraction);
        if (only_in_abstraction == InAbstractionFlawSearchKind::TRUE && backward_flaws.empty()) {
            backward_flaws = get_backward_flaws(solution, true, InAbstractionFlawSearchKind::FALSE);
        }
    }

    return pick_sequence_split(move(forward_flaws), move(backward_flaws), solution, rng);
}

SplitProperties FlawSearch::pick_sequence_split(
    vector<LegacyFlaw> &&forward_flaws,
    vector<LegacyFlaw> &&backward_flaws,
    const Solution &solution,
    utils::RandomNumberGenerator &rng) {
    bool debug = log.is_at_least_debug();
    if (debug) {
        utils::g_log << "Forward splits: " << forward_flaws << endl;
        utils::g_log << "Backward splits: " << backward_flaws << endl;
    }
    SplitProperties best =
        select_from_sequence_flaws(move(forward_flaws), move(backward_flaws), solution, rng);
    if (debug) {
        if (best.split) {
            utils::g_log << "Selected split: " << *best.split << endl;
        } else {
            utils::g_log << "No splits" << endl;
        }
        utils::g_log << "Selected direction: " << (best.backward_direction ? "backward" : "forward") << endl;
    }
    return best;
}

SplitProperties FlawSearch::select_from_sequence_flaws(
    vector<LegacyFlaw> &&forward_flaws,
    vector<LegacyFlaw> &&backward_flaws,
    const Solution &solution,
    utils::RandomNumberGenerator &rng) {
    int n_forward = forward_flaws.size();
    int n_backward = backward_flaws.size();
    if (forward_flaws.empty() && backward_flaws.empty()) {
        return SplitProperties(nullptr, 0, false, 0, 0);
    }
    unique_ptr<Split> best_fw = forward_flaws.empty() ? nullptr
        : select_flaw_and_pick_split(move(forward_flaws), false, rng);
    unique_ptr<Split> best_bw = backward_flaws.empty() ? nullptr
        : select_flaw_and_pick_split(move(backward_flaws), true, rng);

    if (!best_fw) {
        return return_best_sequence_split(move(best_bw), true, n_forward, n_backward, solution);
    } else if (!best_bw) {
        return return_best_sequence_split(move(best_fw), false, n_forward, n_backward, solution);
    } else {
        const AbstractState &fw_abstract_state = abstraction.get_state(best_fw->abstract_state_id);
        const AbstractState &bw_abstract_state = abstraction.get_state(best_bw->abstract_state_id);
        int dist_diff = shortest_paths.get_64bit_goal_distance(best_fw->abstract_state_id) -
            shortest_paths.get_64bit_goal_distance(best_bw->abstract_state_id);

        switch (split_selector.sequence_pick) {
        // Cache is filled only the `default` case.
        case PickSequenceFlaw::RANDOM:
            if (rng.random(2) == 0) {
                return return_best_sequence_split(move(best_fw), false, n_forward, n_backward, solution, false);
            } else {
                return return_best_sequence_split(move(best_bw), true, n_forward, n_backward, solution, false);
            }
        case PickSequenceFlaw::FIRST_FLAW:
            return sequence_splits_tiebreak(move(best_fw),
                                            fw_abstract_state,
                                            move(best_bw),
                                            bw_abstract_state,
                                            n_forward,
                                            n_backward,
                                            solution,
                                            false);
        case PickSequenceFlaw::LAST_FLAW:
            return sequence_splits_tiebreak(move(best_fw),
                                            fw_abstract_state,
                                            move(best_bw),
                                            bw_abstract_state,
                                            n_forward,
                                            n_backward,
                                            solution,
                                            false);
        case PickSequenceFlaw::CLOSEST_TO_GOAL_FLAW:
            // Find which is closest to goal, and break tie only if they are at the same distance.
            if (dist_diff > 0) {
                // Forward distance is higher, return backward flaw.
                return return_best_sequence_split(move(best_bw), true, n_forward, n_backward, solution, false);
            } else if (dist_diff < 0) {
                return return_best_sequence_split(move(best_fw), false, n_forward, n_backward, solution, false);
            } else {
                return sequence_splits_tiebreak(move(best_fw),
                                                fw_abstract_state,
                                                move(best_bw),
                                                bw_abstract_state,
                                                n_forward,
                                                n_backward,
                                                solution,
                                                false);
            }
        default:
            double diff_rate =
                split_selector.rate_split(fw_abstract_state, *best_fw, sequence_to_split(split_selector.sequence_pick)) -
                split_selector.rate_split(bw_abstract_state, *best_bw, sequence_to_split(split_selector.sequence_pick));
            if (abs(diff_rate) < EPSILON) {
                // Break tie because they are equal.
                return sequence_splits_tiebreak(move(best_fw),
                                                fw_abstract_state,
                                                move(best_bw),
                                                bw_abstract_state,
                                                n_forward,
                                                n_backward,
                                                solution);
            } else if (diff_rate > 0) {
                // Forward is higher.
                return return_best_sequence_split(move(best_fw), false, n_forward, n_backward, solution);
            } else {
                return return_best_sequence_split(move(best_bw), true, n_forward, n_backward, solution);
            }
        }
    }
}

SplitProperties FlawSearch::sequence_splits_tiebreak(unique_ptr<Split> best_fw,
                                                     const AbstractState &fw_abstract_state,
                                                     unique_ptr<Split> best_bw,
                                                     const AbstractState &bw_abstract_state,
                                                     int n_forward,
                                                     int n_backward,
                                                     const Solution &solution,
                                                     bool invalidate_cache) {
    double tiebreak_diff_rate =
        split_selector.rate_split(fw_abstract_state, *best_fw, sequence_to_split(split_selector.sequence_tiebreak_pick)) -
        split_selector.rate_split(bw_abstract_state, *best_bw, sequence_to_split(split_selector.sequence_tiebreak_pick));
    if (abs(tiebreak_diff_rate) < EPSILON) {
        // Preference for backward flaw.
        return return_best_sequence_split(move(best_bw), true, n_forward, n_backward, solution, invalidate_cache);
    } else if (tiebreak_diff_rate > 0) {
        return return_best_sequence_split(move(best_fw), false, n_forward, n_backward, solution, invalidate_cache);
    } else {
        return return_best_sequence_split(move(best_bw), true, n_forward, n_backward, solution, invalidate_cache);
    }
}

SplitProperties FlawSearch::return_best_sequence_split(unique_ptr<Split> best,
                                                       bool bw_dir,
                                                       int n_forward,
                                                       int n_backward,
                                                       const Solution &solution,
                                                       bool invalidate_cache) {
    if (invalidate_cache) {
        splits_cache_invalidate(best->abstract_state_id);
    }
    return SplitProperties(move(best), get_plan_perc(best->abstract_state_id, solution), bw_dir, n_forward, n_backward);
}

unique_ptr<Split> FlawSearch::select_flaw_and_pick_split(
    vector<LegacyFlaw> &&flaws,
    bool backward_direction,
    utils::RandomNumberGenerator &rng) {
    assert(!flaws.empty());
    if (flaws.size() == 1) {
        return get_split_from_flaw(move(flaws[0]), backward_direction, backward_direction);
    } else {
        unique_ptr<Split> selected_split = nullptr;
        switch (split_selector.sequence_pick) {
        case PickSequenceFlaw::RANDOM:
            return get_split_from_flaw(move(*rng.choose(flaws)), backward_direction, backward_direction);
        case PickSequenceFlaw::FIRST_FLAW:
            return get_split_from_flaw(move(flaws[0]), backward_direction, backward_direction);
        case PickSequenceFlaw::LAST_FLAW:
            return get_split_from_flaw(move(flaws.back()), backward_direction, backward_direction);
        case PickSequenceFlaw::CLOSEST_TO_GOAL_FLAW:
            if (backward_direction) {
                return get_split_from_flaw(move(flaws[0]), backward_direction, backward_direction);
            } else {
                return get_split_from_flaw(move(flaws.back()), backward_direction, backward_direction);
            }
        default:
            double max_rating = numeric_limits<double>::lowest();
            double max_tiebreak_rating = numeric_limits<double>::lowest();
            for (LegacyFlaw &fl : flaws) {
                unique_ptr<Split> split = splits_cache_get(move(fl), backward_direction, backward_direction);
                const AbstractState &abs = abstraction.get_state(split->abstract_state_id);
                double rating = split_selector.rate_split(abs, *split, sequence_to_split(split_selector.sequence_pick));
                if (rating > max_rating) {
                    max_rating = rating;
                    max_tiebreak_rating = split_selector.rate_split(abs, *split, sequence_to_split(split_selector.sequence_tiebreak_pick));
                    selected_split = move(split);
                } else if (max_rating - rating < EPSILON) {
                    double tiebreak_rating = split_selector.rate_split(abs, *split, sequence_to_split(split_selector.sequence_tiebreak_pick));
                    if (tiebreak_rating > max_tiebreak_rating) {
                        max_rating = rating;
                        max_tiebreak_rating = tiebreak_rating;
                        selected_split = move(split);
                    }
                }
            }
            assert(selected_split);
            return selected_split;
        }
    }
}
}