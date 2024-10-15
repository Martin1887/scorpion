#include "cegar.h"
#include "flaw_search.h"

#include "abstraction.h"
#include "abstract_state.h"
#include "flaw.h"
#include "shortest_paths.h"
#include "split_selector.h"
#include "transition_system.h"
#include "utils.h"

#include "../task_utils/cartesian_set_facts_proxy_iterator.h"
#include "../task_utils/successor_generator.h"
#include "../task_utils/task_properties.h"
#include "../utils/countdown_timer.h"
#include "../utils/rng.h"

#include <iterator>
#include <locale>

using namespace std;

namespace cartesian_abstractions {
void FlawSearch::push_flaw_if_not_filtered(vector<LegacyFlaw> &flaws,
                                           const LegacyFlaw &flaw,
                                           const Solution &solution,
                                           const bool backward_direction,
                                           unique_ptr<LegacyFlaw> &first_filtered_flaw,
                                           bool force_push) {
    if (split_selector.filter_pick != FilterSplit::NONE) {
        // Remove the split if it is filtered, the computation of the best split
        // of the abstract state is cached, so no penalties for no filtered flaws.
        unique_ptr<Split> best = get_split_from_flaw(flaw,
                                                     get_optimal_plan_cost(solution, task_proxy),
                                                     backward_direction,
                                                     backward_direction);
        if (force_push || !best->is_filtered) {
            flaws.push_back(move(flaw));
        } else if (!first_filtered_flaw) {
            first_filtered_flaw = make_unique<LegacyFlaw>(move(flaw));
        }
    } else {
        flaws.push_back(move(flaw));
    }
}

unique_ptr<Split> FlawSearch::last_not_filtered_flaw(vector<LegacyFlaw> &flaws,
                                                     Cost solution_cost,
                                                     const bool backward_direction) {
    unique_ptr<Split> best_split = nullptr;
    unique_ptr<Split> actual_last = nullptr;
    do {
        LegacyFlaw last = flaws.back();
        flaws.pop_back();
        best_split = get_split_from_flaw(last, solution_cost, backward_direction, backward_direction);
        if (best_split->is_filtered && !actual_last) {
            actual_last = move(best_split);
        }
    } while ((!best_split || best_split->is_filtered) && !flaws.empty());
    if (!best_split || best_split->is_filtered) {
        best_split = move(actual_last);
    }
    return best_split;
}

tuple<CartesianState, int> FlawSearch::first_flaw_search_state(const Solution &solution,
                                                               InAbstractionFlawSearchKind only_in_abstraction,
                                                               const AbstractState * &abstract_state) {
    int start_abstract_state_index = 0;
    switch (only_in_abstraction) {
    case InAbstractionFlawSearchKind::TRUE:
        return {CartesianState(abstraction.get_initial_state()), start_abstract_state_index};
    case InAbstractionFlawSearchKind::FALSE:
        return {CartesianState(get_domain_sizes(task_proxy),
                               task_properties::get_fact_pairs(state_registry->get_initial_state())),
                start_abstract_state_index};
    case InAbstractionFlawSearchKind::ITERATIVE_IN_REGRESSION:
        // In goal state and in the last state before goal no possible flaw
        // exists, so get the previous state, or the initial state if 2 or
        // fewer states.
        if (solution.size() >= 3) {
            abstract_state = &abstraction.get_state(solution.at(solution.size() - 3).target_id);
            start_abstract_state_index = solution.size() - 2;
            return {CartesianState(abstraction.get_state(solution.at(solution.size() - 3).target_id)),
                    start_abstract_state_index};
        } else if (solution.size() == 2) {
            start_abstract_state_index = 0;
            return {CartesianState(abstraction.get_initial_state()), start_abstract_state_index};
        } else {
            start_abstract_state_index = -1;
            return {CartesianState(get_domain_sizes(task_proxy),
                                   task_properties::get_fact_pairs(state_registry->get_initial_state())),
                    start_abstract_state_index};
        }
    }
    // This cannot happen, all cases are handled above.
    throw utils::Exception("Reached unreachable code");
}

void FlawSearch::get_deviation_splits(
    const AbstractState &abs_state,
    const vector<reference_wrapper<const CartesianState>> &flaw_search_states,
    const AbstractState &target_abs_state,
    const vector<int> &domain_sizes,
    const disambiguation::DisambiguatedOperator &op,
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
    const CartesianSet &target_set = target_abs_state.get_cartesian_set();
    const CartesianSet &pre = op.get_precondition().get_cartesian_set();
    int op_cost = op.get_cost();
    int biggest_var_size = 0;
    int n_vars = target_set.get_n_vars();
    for (int var = 0; var < n_vars; var++) {
        if (!op.has_effect(var) && domain_sizes[var] > biggest_var_size) {
            biggest_var_size = domain_sizes[var];
        }
    }
    // Create the vectors in the heap only once and reuse it for all vars.
    vector<int> wanted;
    wanted.reserve(biggest_var_size);
    vector<bool> var_intersects;
    bool multiple_states = flaw_search_states.size() > 1;
    if (multiple_states) {
        var_intersects = vector<bool>(flaw_search_states.size(), false);
    }
    for (int var = 0; var < n_vars; var++) {
        if (!op.has_effect(var)) {
            bool var_intersects_in_state;
            if (multiple_states) {
                int i = 0;
                for (auto &fs_state : flaw_search_states) {
                    var_intersects[i] = target_set.intersects_intersection(fs_state.get().get_cartesian_set(), pre, var);
                    i++;
                }
            } else {
                var_intersects_in_state = target_set.intersects_intersection(flaw_search_states[0].get().get_cartesian_set(), pre, var);
            }
            bool wanted_computed = false;
            for (int value = 0; value < domain_sizes[var]; ++value) {
                int count = 0;
                int i = 0;
                for (auto &fs_state : flaw_search_states) {
                    // In regression the value may be not in the target state or in
                    // the precondition to get a deviation,
                    // e.g.: (1,2,3), pre: (1,2,5), post: (1,2,5), fs_state: (3,5),
                    // 3 is not in the precondition and it is a deviation despite
                    // being in the target state.
                    bool var_intersects_in_this;
                    if (multiple_states) {
                        var_intersects_in_this = var_intersects[i];
                    } else {
                        var_intersects_in_this = var_intersects_in_state;
                    }
                    bool target_contains = target_set.test(var, value) && pre.test(var, value);
                    if (!var_intersects_in_this && !target_contains && fs_state.get().includes(var, value) && abs_state.includes(var, value)) {
                        ++count;
                    }
                    i++;
                }
                if (count) {
                    if (!wanted_computed) {
                        wanted_computed = true;
                        wanted.clear();
                        for (int value = 0; value < domain_sizes[var]; ++value) {
                            if (abs_state.includes(var, value) &&
                                pre.test(var, value) &&
                                target_abs_state.includes(var, value)) {
                                wanted.push_back(value);
                            }
                        }
                    }
                    assert(!wanted.empty());
                    if (split_unwanted_values) {
                        FlawSearch::add_split(splits, Split(
                                                  abs_state.get_id(), var, -1, {value},
                                                  count, op_cost), true);
                    } else {
                        FlawSearch::add_split(splits, Split(
                                                  abs_state.get_id(), var, value, wanted,
                                                  count, op_cost));
                    }
                }
            }
        }
    }
}

unique_ptr<Split> FlawSearch::create_split(
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

    const TransitionSystem &ts = abstraction.get_transition_system();
    vector<vector<Split>> splits;
    // Splits are grouped by variable only if split by wanted values.
    if (split_unwanted_values) {
        splits = vector<vector<Split>>();
    } else {
        splits = vector<vector<Split>>(task_proxy.get_variables().size());
    }
    // Create the vectors only once to save memory allocations and set values in each iter.
    vector<bool> applicable(states.size(), true);
    vector<bool> var_intersects(states.size(), true);
    for (auto &pair : get_f_optimal_transitions(abstract_state_id)) {
        fill(applicable.begin(), applicable.end(), true);
        int op_id = pair.first;
        const vector<int> &targets = pair.second;
        const disambiguation::DisambiguatedOperator &op = (*ts.get_operators())[op_id];

        const CartesianSet &pre = op.get_precondition().get_cartesian_set();
        const CartesianSet &abstract_state_set = abstract_state.get_cartesian_set();
        int n_vars = pre.get_n_vars();
        for (int var = 0; var < n_vars; var++) {
            int i = 0;
            for (const CartesianState &state : states) {
                var_intersects[i] = pre.intersects(state.get_cartesian_set(), var);
                if (!var_intersects[i]) {
                    applicable[i] = false;
                }
                i++;
            }
            for (int value = 0; value < domain_sizes[var]; ++value) {
                int count = 0;
                int i = 0;
                for (const CartesianState &state : states) {
                    if (!var_intersects[i] &&
                        state.includes(var, value) &&
                        abstract_state.includes(var, value)) {
                        count++;
                    }
                    i++;
                }
                if (count) {
                    assert(!pre.test(var, value));
                    if (split_unwanted_values) {
                        add_split(splits, Split(
                                      abstract_state_id, var, -1,
                                      {value}, count,
                                      op.get_cost()), true);
                    } else {
                        add_split(splits, Split(
                                      abstract_state_id, var, value,
                                      pre.get_intersection_values(var, abstract_state_set), count,
                                      op.get_cost()));
                    }
                }
            }
        }

        phmap::flat_hash_map<int, vector<reference_wrapper<const CartesianState>>> deviation_states_by_target;
        for (size_t i = 0; i < states.size(); ++i) {
            // Retrieving deviation flaws on states with inapplicable flaws work worse.
            if (!applicable[i] /*&& !in_sequence*/) {
                continue;
            }
            const CartesianState &state = states[i];
            if (!in_sequence) {
                assert(state.is_applicable(op));
            }
            bool target_hit = false;
            for (int target : targets) {
                if (!utils::extra_memory_padding_is_reserved()) {
                    return nullptr;
                }

                // At most one of the f-optimal targets can include the successor state.
                if (!target_hit &&
                    ((applicable[i] && state.reach_with_op(abstraction.get_state(target), op)) ||
                     (!applicable[i] && state.reach_with_inapplicable_op(abstraction.get_state(target), op)))) {
                    // No flaw
                    target_hit = true;
                } else {
                    // Deviation flaw
                    deviation_states_by_target[target].push_back(ref(state));
                }
            }
        }

        for (auto &&[target, deviation_states] : deviation_states_by_target) {
            if (!deviation_states.empty()) {
                get_deviation_splits(
                    abstract_state, deviation_states,
                    abstraction.get_state(target), domain_sizes, op,
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

unique_ptr<Split> FlawSearch::create_split_from_goal_state(
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
                        if (value != goal_value && abstract_state.includes(var, value)) {
                            other_values.push_back(value);
                        }
                    }

                    if (split_unwanted_values) {
                        for (const CartesianState &state : states) {
                            for (auto &&[fact_var, fact_value] : state.get_cartesian_set().iter(var)) {
                                if (fact_value != goal_value && abstract_state.includes(var, fact_value)) {
                                    if (log.is_at_least_debug()) {
                                        log << "add_split(var " << var << ", val " << fact_value
                                            << "!=" << goal_value << ")" << endl;
                                    }
                                    add_split(splits, Split(
                                                  abstract_state_id, var, goal_value,
                                                  {fact_value}, 1), true);
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
    Split split = split_selector.pick_split(abstract_state, move(splits), solution_cost, rng);
    pick_split_timer.stop();
    return utils::make_unique_ptr<Split>(move(split));
}

vector<LegacyFlaw> FlawSearch::get_forward_flaws(const Solution &solution,
                                                 const InAbstractionFlawSearchKind only_in_abstraction) {
    vector<LegacyFlaw> flaws{};
    state_registry = utils::make_unique_ptr<StateRegistry>(task_proxy);
    bool debug = log.is_at_least_debug();
    bool force_push_filtered_flaws = true;
    // This pointer is used to return the first found flaw if all have been
    // filtered.
    unique_ptr<LegacyFlaw> first_filtered_flaw = nullptr;
    // Only avoid pushing filtered flaws in sequence strategies that stop at the
    // first flaw.
    if (only_in_abstraction == InAbstractionFlawSearchKind::ITERATIVE_IN_REGRESSION ||
        (in_sequence && split_selector.sequence_pick == PickSequenceFlaw::FIRST_FLAW)) {
        force_push_filtered_flaws = false;
    }
    if (debug)
        log << "Check solution:" << endl;

    const AbstractState *abstract_state = &abstraction.get_initial_state();
    auto [ flaw_search_state, start_abstract_state_index ] =
        first_flaw_search_state(solution,
                                only_in_abstraction,
                                abstract_state);

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
            const Transition &step = solution.at(i);
            const disambiguation::DisambiguatedOperator &op = (*abstraction.get_transition_system().get_operators())[step.op_id];
            const AbstractState *next_abstract_state = &abstraction.get_state(step.target_id);
            if (flaw_search_state.is_applicable(op)) {
                if (debug) {
                    log << endl << "  Move to " << *next_abstract_state << " with "
                        << op.get_name() << endl;
                }
                if (!flaw_search_state.reach_with_op(*next_abstract_state, op)) {
                    if (debug) {
                        log << "  Paths deviate." << endl;
                        log << "  Previous flaw-search state: " << flaw_search_state << endl;
                        log << "  Previous abstract state: " << *abstract_state << endl;
                        log << "  Op pre: " << op.get_precondition() << endl << "  Op post: " << op.get_post() << endl;
                    }
                    push_flaw_if_not_filtered(flaws,
                                              LegacyFlaw(flaw_search_state,
                                                         abstract_state->get_id(),
                                                         false),
                                              solution,
                                              false,
                                              first_filtered_flaw,
                                              force_push_filtered_flaws);
                    flaw_search_state.progress(op);
                    if (debug) {
                        log << "  Flaw-search state: " << flaw_search_state << endl;
                    }
                    if (!in_sequence ||
                        (split_selector.sequence_pick == PickSequenceFlaw::FIRST_FLAW && !flaws.empty())) {
                        return flaws;
                    } else {
                        flaw_search_state.undeviate(*next_abstract_state);
                        if (debug) {
                            log << "  Undeviated state: " << flaw_search_state << endl;
                            log << "  Abstract state: " << *next_abstract_state << endl;
                        }
                    }
                } else {
                    flaw_search_state.progress(op);
                }
                abstract_state = next_abstract_state;
            } else {
                if (debug) {
                    log << "  Operator not applicable: " << op.get_name() << endl;
                    log << "  Operator preconditions: " << op.get_precondition().get_cartesian_set() << endl;
                    log << "  Abstract state: " << *abstract_state << endl;
                    log << "  Flaw-search state: " << flaw_search_state << endl;
                }
                push_flaw_if_not_filtered(flaws,
                                          LegacyFlaw(flaw_search_state,
                                                     abstract_state->get_id(),
                                                     false),
                                          solution,
                                          false,
                                          first_filtered_flaw,
                                          force_push_filtered_flaws);
                if (!in_sequence ||
                    (split_selector.sequence_pick == PickSequenceFlaw::FIRST_FLAW && !flaws.empty())) {
                    return flaws;
                } else {
                    abstract_state = &abstraction.get_state(step.target_id);
                    // Apply the operator as if it were applicable (and undeviate if needed).
                    flaw_search_state.progress(op);
                    if (!abstract_state->intersects(flaw_search_state)) {
                        if (debug) {
                            log << "  The state " << flaw_search_state << " does not intersects" << endl;
                            log << "  Abstract state: " << *abstract_state << endl;
                            log << "  Op pre: " << op.get_precondition() << ", op post: " << op.get_post() << endl;
                        }
                        flaw_search_state.undeviate(*abstract_state);
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
                push_flaw_if_not_filtered(flaws,
                                          LegacyFlaw(move(flaw_search_state),
                                                     abstract_state->get_id(),
                                                     true),
                                          solution,
                                          false,
                                          first_filtered_flaw,
                                          force_push_filtered_flaws);
            }
        }
        if (only_in_abstraction == InAbstractionFlawSearchKind::ITERATIVE_IN_REGRESSION && flaws.empty()) {
            if (start_abstract_state_index < 0) {
                break;
            } else if (start_abstract_state_index == 0) {
                flaw_search_state = CartesianState(get_domain_sizes(task_proxy),
                                                   task_properties::get_fact_pairs(state_registry->get_initial_state()));
                abstract_state = &abstraction.get_initial_state();
            } else if (start_abstract_state_index == 1) {
                flaw_search_state = CartesianState(abstraction.get_initial_state());
                abstract_state = &abstraction.get_initial_state();
            } else {
                flaw_search_state = CartesianState(abstraction.get_state(solution.at(start_abstract_state_index - 2).target_id));
                abstract_state = &abstraction.get_state(solution.at(start_abstract_state_index - 2).target_id);
            }
        }
        start_abstract_state_index--;
    } while (only_in_abstraction == InAbstractionFlawSearchKind::ITERATIVE_IN_REGRESSION && flaws.empty());

    if (flaws.empty() && first_filtered_flaw) {
        flaws.push_back(move(*first_filtered_flaw));
    }

    return flaws;
}

vector<LegacyFlaw> FlawSearch::get_backward_flaws(const Solution &solution,
                                                  const InAbstractionFlawSearchKind only_in_abstraction) {
    vector<LegacyFlaw> flaws{};
    bool force_push_filtered_flaws = true;
    // This pointer is used to return the first found flaw if all have been
    // filtered.
    unique_ptr<LegacyFlaw> first_filtered_flaw = nullptr;
    // Only avoid pushing filtered flaws in sequence strategies that stop at the
    // first flaw.
    if (only_in_abstraction == InAbstractionFlawSearchKind::ITERATIVE_IN_REGRESSION ||
        (in_sequence &&
         (split_selector.sequence_pick == PickSequenceFlaw::FIRST_FLAW ||
          split_selector.sequence_pick == PickSequenceFlaw::CLOSEST_TO_GOAL_FLAW))) {
        force_push_filtered_flaws = false;
    }
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
    CartesianState flaw_search_state = only_in_abstraction != InAbstractionFlawSearchKind::FALSE ?
        CartesianState(abstract_state->clone_cartesian_set())
        :
        CartesianState(get_domain_sizes(task_proxy), move(goals_facts));
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
        const Transition &step = solution.at(i);
        const disambiguation::DisambiguatedOperator &op = (*abstraction.get_transition_system().get_operators())[step.op_id];
        if (flaw_search_state.is_backward_applicable(op)) {
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
                log << "  Op pre: " << op.get_precondition() << endl << "  Op post: " << op.get_post() << endl;
            }
            if (!flaw_search_state.reach_backwards_with_op(*next_abstract_state, op)) {
                if (debug) {
                    log << "  Paths deviate." << endl;
                    log << "  Flaw-search state: " << flaw_search_state << endl;
                    log << "  Previous abstract state: " << *abstract_state << endl;
                    log << "  Op pre: " << op.get_precondition() << endl << "  Op post: " << op.get_post() << endl;
                    log << "  Abstract state: " << *next_abstract_state << endl;
                }
                push_flaw_if_not_filtered(flaws,
                                          LegacyFlaw(flaw_search_state,
                                                     abstract_state->get_id(),
                                                     false),
                                          solution,
                                          true,
                                          first_filtered_flaw,
                                          force_push_filtered_flaws);
                flaw_search_state.regress(op);
                if (debug) {
                    log << "  In flaw-search space move to "
                        << flaw_search_state << " with " << op.get_name() << endl;
                }
                if (!in_sequence ||
                    (!flaws.empty() &&
                     (only_in_abstraction == InAbstractionFlawSearchKind::ITERATIVE_IN_REGRESSION ||
                      split_selector.sequence_pick == PickSequenceFlaw::FIRST_FLAW ||
                      split_selector.sequence_pick == PickSequenceFlaw::CLOSEST_TO_GOAL_FLAW))) {
                    return flaws;
                } else {
                    flaw_search_state.undeviate(*next_abstract_state);
                    if (debug) {
                        log << "  Undeviated state: " << flaw_search_state << endl;
                    }
                }
            } else {
                flaw_search_state.regress(op);
                if (debug) {
                    log << "  In flaw-search space move to "
                        << flaw_search_state << " with " << op.get_name() << endl;
                }
            }
            abstract_state = next_abstract_state;
            if (intersect_flaw_search_abstract_states) {
                flaw_search_state = flaw_search_state.intersection(*abstract_state);
                if (debug)
                    log << "  Intersected flaw-search state: " << flaw_search_state << endl;
            }
        } else {
            if (debug)
                log << "  Operator not backward applicable: " << op.get_name() << endl;
            push_flaw_if_not_filtered(flaws,
                                      LegacyFlaw(flaw_search_state,
                                                 abstract_state->get_id(),
                                                 false),
                                      solution,
                                      true,
                                      first_filtered_flaw,
                                      force_push_filtered_flaws);
            if (!in_sequence ||
                (!flaws.empty() &&
                 (only_in_abstraction == InAbstractionFlawSearchKind::ITERATIVE_IN_REGRESSION ||
                  split_selector.sequence_pick == PickSequenceFlaw::FIRST_FLAW ||
                  split_selector.sequence_pick == PickSequenceFlaw::CLOSEST_TO_GOAL_FLAW))) {
                return flaws;
            } else {
                if (i > 0) {
                    abstract_state = &abstraction.get_state(solution.at(i - 1).target_id);
                } else {
                    abstract_state = initial_abstract_state;
                }
                // Apply the operator as if it were applicable (and undeviate if needed).
                flaw_search_state.regress(op);
                if (!abstract_state->intersects(flaw_search_state)) {
                    if (debug) {
                        log << "  The state " << flaw_search_state << " does not intersects" << endl;
                        log << "  Abstract state: " << *abstract_state << endl;
                    }
                    flaw_search_state.undeviate(*abstract_state);
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
            push_flaw_if_not_filtered(flaws,
                                      LegacyFlaw(move(flaw_search_state),
                                                 abstract_state->get_id(),
                                                 true),
                                      solution,
                                      true,
                                      first_filtered_flaw,
                                      force_push_filtered_flaws);
        }
    }

    // It could happen that flaws are not found because starting in the goals
    // abstract state, so start from the goals.
    if (only_in_abstraction == InAbstractionFlawSearchKind::ITERATIVE_IN_REGRESSION && flaws.empty()) {
        flaws = get_backward_flaws(solution, InAbstractionFlawSearchKind::FALSE);
    }

    if (flaws.empty() && first_filtered_flaw) {
        flaws.push_back(move(*first_filtered_flaw));
    }

    return flaws;
}

SplitProperties FlawSearch::get_sequence_splits(const Solution &solution) {
    assert(forward_direction || backward_direction);
    vector<LegacyFlaw> forward_flaws{};
    vector<LegacyFlaw> backward_flaws{};
    if (!in_batch || sequence_flaws_queue.empty()) {
        if (forward_direction) {
            forward_flaws = get_forward_flaws(solution, only_in_abstraction);
            if (only_in_abstraction == InAbstractionFlawSearchKind::TRUE && forward_flaws.empty()) {
                forward_flaws = get_forward_flaws(solution, InAbstractionFlawSearchKind::FALSE);
            }
        }
        if (backward_direction) {
            backward_flaws = get_backward_flaws(solution, only_in_abstraction);
            if (only_in_abstraction == InAbstractionFlawSearchKind::TRUE && backward_flaws.empty()) {
                backward_flaws = get_backward_flaws(solution, InAbstractionFlawSearchKind::FALSE);
            }
        }
    }
    if (in_batch) {
        if (sequence_flaws_queue.empty()) {
            // Move backward_flaws into forward_flaws for a more performant
            // operatiion and then move it into sequence_flaws_stack.
            forward_flaws.insert(forward_flaws.end(),
                                 make_move_iterator(backward_flaws.begin()),
                                 make_move_iterator(backward_flaws.end()));
            sequence_flaws_queue.insert(sequence_flaws_queue.end(),
                                        make_move_iterator(forward_flaws.begin()),
                                        make_move_iterator(forward_flaws.end()));
        }
        // Currently only forward or backward are possible for batched splits,
        // another attribute would be needed for getting the direction in
        // bidirectional batched splits.
        vector<LegacyFlaw> flaws = vector<LegacyFlaw>();
        if (!sequence_flaws_queue.empty()) {
            flaws = vector<LegacyFlaw>{move(sequence_flaws_queue.front())};
            sequence_flaws_queue.pop_front();
        }
        if (forward_direction) {
            return pick_sequence_split(move(flaws), vector<LegacyFlaw>{}, solution, rng);
        } else {
            return pick_sequence_split(vector<LegacyFlaw>{}, move(flaws), solution, rng);
        }
    } else {
        return pick_sequence_split(move(forward_flaws), move(backward_flaws), solution, rng);
    }
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
    Cost solution_cost = get_optimal_plan_cost(solution, task_proxy);
    unique_ptr<Split> best_fw = forward_flaws.empty() ? nullptr
        : select_flaw_and_pick_split(move(forward_flaws), false, solution_cost, rng);
    unique_ptr<Split> best_bw = backward_flaws.empty() ? nullptr
        : select_flaw_and_pick_split(move(backward_flaws), true, solution_cost, rng);

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
        case PickSequenceFlaw::RANDOM:
            if (rng.random(2) == 0) {
                return return_best_sequence_split(move(best_fw), false, n_forward, n_backward, solution);
            } else {
                return return_best_sequence_split(move(best_bw), true, n_forward, n_backward, solution);
            }
        case PickSequenceFlaw::CLOSEST_TO_GOAL_FLAW:
            // Find which is closest to goal, and break tie only if they are at the same distance.
            if (dist_diff > 0) {
                // Forward distance is higher, return backward flaw.
                return return_best_sequence_split(move(best_bw), true, n_forward, n_backward, solution);
            } else if (dist_diff < 0) {
                return return_best_sequence_split(move(best_fw), false, n_forward, n_backward, solution);
            } else {
                return sequence_splits_tiebreak(move(best_fw),
                                                fw_abstract_state,
                                                move(best_bw),
                                                bw_abstract_state,
                                                n_forward,
                                                n_backward,
                                                solution);
            }
        default:
            Cost solution_cost = get_optimal_plan_cost(solution, task_proxy);
            double diff_rate =
                split_selector.rate_split(fw_abstract_state, *best_fw, split_selector.first_pick, solution_cost) -
                split_selector.rate_split(bw_abstract_state, *best_bw, split_selector.first_pick, solution_cost);
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
    Cost solution_cost = get_optimal_plan_cost(solution, task_proxy);
    double tiebreak_diff_rate =
        split_selector.rate_split(fw_abstract_state, *best_fw, split_selector.tiebreak_pick, solution_cost) -
        split_selector.rate_split(bw_abstract_state, *best_bw, split_selector.tiebreak_pick, solution_cost);
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
    Cost solution_cost,
    utils::RandomNumberGenerator &rng) {
    assert(!flaws.empty());
    if (flaws.size() == 1) {
        return get_split_from_flaw(move(flaws[0]), solution_cost, backward_direction, backward_direction);
    } else {
        unique_ptr<Split> selected_split = nullptr;
        // Splits cache is used only when needed (the same flaw can happen in
        // another iteration or filtered status must be checked).
        switch (split_selector.sequence_pick) {
        // The first flaw (and closest to goal in the backward direction) is
        // already filtered, but for the other cases iteration is needed
        // until getting a non-filtered flaw.
        // In default case this is automatically handled by the lowest
        // rating of filtered flaws.
        case PickSequenceFlaw::RANDOM:
        {
            unique_ptr<Split> best_split = nullptr;
            unique_ptr<Split> first_filtered = nullptr;
            do {
                auto flaws_iterator = flaws.begin() + rng.random(flaws.size());
                best_split = get_split_from_flaw(*flaws_iterator, solution_cost, backward_direction, backward_direction);
                flaws.erase(flaws_iterator);
                if (best_split->is_filtered && !first_filtered) {
                    first_filtered = move(best_split);
                }
            } while ((!best_split || best_split->is_filtered) && !flaws.empty());
            if (!best_split || best_split->is_filtered) {
                best_split = move(first_filtered);
            }
            return best_split;
        }
        case PickSequenceFlaw::FIRST_FLAW:
            return get_split_from_flaw(flaws[0], solution_cost, backward_direction, backward_direction);
        case PickSequenceFlaw::LAST_FLAW:
            return last_not_filtered_flaw(flaws, solution_cost, backward_direction);
        case PickSequenceFlaw::CLOSEST_TO_GOAL_FLAW:
            if (backward_direction) {
                return get_split_from_flaw(flaws[0], solution_cost, backward_direction, backward_direction);
            } else {
                return last_not_filtered_flaw(flaws, solution_cost, backward_direction);
            }
        default:
            double max_rating = numeric_limits<double>::lowest();
            double max_tiebreak_rating = numeric_limits<double>::lowest();
            for (LegacyFlaw &fl : flaws) {
                unique_ptr<Split> split = get_split_from_flaw(move(fl), solution_cost, backward_direction, backward_direction);
                const AbstractState &abs = abstraction.get_state(split->abstract_state_id);
                double rating = split_selector.rate_split(abs, *split, split_selector.first_pick, solution_cost);
                if (rating > max_rating) {
                    max_rating = rating;
                    max_tiebreak_rating = split_selector.rate_split(abs, *split, split_selector.tiebreak_pick, solution_cost);
                    selected_split = move(split);
                } else if (max_rating - rating < EPSILON) {
                    double tiebreak_rating = split_selector.rate_split(abs, *split, split_selector.tiebreak_pick, solution_cost);
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
