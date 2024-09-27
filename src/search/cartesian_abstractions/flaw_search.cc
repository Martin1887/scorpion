#include "flaw_search.h"

#include "abstraction.h"
#include "abstract_state.h"
#include "cegar.h"
#include "flaw.h"
#include "shortest_paths.h"
#include "split_selector.h"
#include "transition_system.h"
#include "utils.h"

#include "../plugins/plugin.h"
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
OptimalTransitions FlawSearch::get_f_optimal_transitions(const Abstraction &abstraction,
                                                         const ShortestPaths &shortest_paths,
                                                         int abstract_state_id) {
    OptimalTransitions transitions;
    for (const Transition &t :
         abstraction.get_transition_system().get_outgoing_transitions()[abstract_state_id]) {
        if (shortest_paths.is_optimal_transition(abstract_state_id, t.op_id, t.target_id)) {
            transitions[t.op_id].push_back(t.target_id);
        }
    }
    return transitions;
}

OptimalTransitions FlawSearch::get_f_optimal_incoming_transitions(const Abstraction &abstraction,
                                                                  const ShortestPaths &shortest_paths,
                                                                  int abstract_state_id) {
    OptimalTransitions transitions;
    for (const Transition &t :
         abstraction.get_transition_system().get_incoming_transitions()[abstract_state_id]) {
        if (shortest_paths.is_optimal_transition(t.target_id, t.op_id, abstract_state_id)) {
            transitions[t.op_id].push_back(t.target_id);
        }
    }
    return transitions;
}

OptimalTransitions FlawSearch::get_f_optimal_backward_transitions(const Abstraction &abstraction,
                                                                  const ShortestPaths &shortest_paths,
                                                                  int abstract_state_id) {
    OptimalTransitions transitions;
    for (const Transition &t :
         abstraction.get_transition_system().get_incoming_transitions()[abstract_state_id]) {
        if (shortest_paths.is_backward_optimal_transition(abstract_state_id, t.op_id, t.target_id)) {
            transitions[t.op_id].push_back(t.target_id);
        }
    }
    return transitions;
}

int FlawSearch::get_abstract_state_id(const State &state) const {
    return abstraction.get_abstract_state_id(state);
}

Cost FlawSearch::get_h_value(int abstract_state_id) const {
    return shortest_paths.get_64bit_goal_distance(abstract_state_id);
}

OptimalTransitions FlawSearch::get_f_optimal_transitions(int abstract_state_id) const {
    return get_f_optimal_transitions(abstraction, shortest_paths, abstract_state_id);
}

OptimalTransitions FlawSearch::get_f_optimal_incoming_transitions(int abstract_state_id) const {
    return get_f_optimal_incoming_transitions(abstraction, shortest_paths, abstract_state_id);
}

OptimalTransitions FlawSearch::get_f_optimal_backward_transitions(int abstract_state_id) const {
    return get_f_optimal_backward_transitions(abstraction, shortest_paths, abstract_state_id);
}

void FlawSearch::add_flaw(int abs_id, const State &state) {
    assert(abstraction.get_state(abs_id).includes(state));

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
    state_registry = utils::make_unique_ptr<StateRegistry>(task_proxy);
    search_space = utils::make_unique_ptr<SearchSpace>(*state_registry, silent_log);
    cached_abstract_state_ids = utils::make_unique_ptr<PerStateInformation<int>>(MISSING);

    assert(flawed_states.empty());

    const State &initial_state = state_registry->get_initial_state();
    (*cached_abstract_state_ids)[initial_state] = abstraction.get_initial_state().get_id();
    SearchNode node = search_space->get_node(initial_state);
    node.open_initial();
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

        disambiguation::DisambiguatedOperator op = (*abstraction.get_transition_system().get_operators())[op_id];

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
                }
                if (pick_flawed_abstract_state == PickFlawedAbstractState::FIRST) {
                    return FAILED;
                }
            } else if (succ_node.is_new()) {
                // No flaw
                (*cached_abstract_state_ids)[succ_state] = target;
                succ_node.open(node, op.get_operator(), op.get_cost());
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

void FlawSearch::add_split(vector<vector<Split>> &splits, Split &&new_split,
                           bool split_unwanted_values) {
    // When splitting by unwanted values splits cannot be grouped by variable.
    if (split_unwanted_values) {
        splits.push_back({move(new_split)});
    } else {
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
}

vector<int> FlawSearch::get_unaffected_variables(
    const disambiguation::DisambiguatedOperator &op, int num_variables) {
    vector<bool> affected(num_variables);
    for (const FactPair &effect : op.get_effects()) {
        affected[effect.var] = true;
    }
    const CartesianSet &pre = op.get_precondition().get_cartesian_set();
    vector<int> unaffected_vars;
    unaffected_vars.reserve(num_variables);
    for (int var = 0; var < num_variables; ++var) {
        // If only one value is possible as precondition the var is affected
        // (as for non-disambiguated operators).
        if (!affected[var] && pre.count(var) != 1) {
            unaffected_vars.push_back(var);
        }
    }
    return unaffected_vars;
}

// TODO: Add comment about split considering multiple transitions.
unique_ptr<Split> FlawSearch::create_split(
    const vector<StateID> &state_ids, int abstract_state_id, Cost solution_cost, bool split_unwanted_values) {
    compute_splits_timer.resume();
    const AbstractState &abstract_state = abstraction.get_state(abstract_state_id);

    if (log.is_at_least_debug()) {
        log << endl;
        log << "Create split for abstract state " << abstract_state_id << " and "
            << state_ids.size() << " concrete states." << endl;
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
        const disambiguation::DisambiguatedOperator &op = (*ts.get_operators())[op_id];

        vector<State> states;
        states.reserve(state_ids.size());
        for (const StateID &state_id : state_ids) {
            states.push_back(state_registry->lookup_state(state_id));
            assert(abstract_state.includes(states.back()));
        }

        vector<bool> applicable(states.size(), true);
        const CartesianSet &pre = op.get_precondition().get_cartesian_set();
        int n_vars = pre.n_vars();
        for (int var = 0; var < n_vars; var++) {
            vector<int> state_value_count(domain_sizes[var], 0);
            for (size_t i = 0; i < states.size(); ++i) {
                const State &state = states[i];
                int state_value = state[var].get_value();
                if (!pre.test(var, state_value)) {
                    // Applicability flaw
                    applicable[i] = false;
                    ++state_value_count[state_value];
                }
            }
            for (int value = 0; value < domain_sizes[var]; ++value) {
                if (state_value_count[value] > 0) {
                    assert(!pre.test(var, value));
                    if (split_unwanted_values) {
                        for (auto &&[fact_var, fact_value] : pre.iter(var)) {
                            add_split(splits, Split(
                                          abstract_state_id, var, fact_value,
                                          {value}, state_value_count[value],
                                          op.get_cost()), true);
                        }
                    } else {
                        add_split(splits, Split(
                                      abstract_state_id, var, value,
                                      pre.get_values(var), state_value_count[value],
                                      op.get_cost()));
                    }
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
                get_deviation_splits(
                    abstract_state, deviation_states,
                    get_unaffected_variables(op, num_vars),
                    abstraction.get_state(target), domain_sizes, splits,
                    split_unwanted_values);
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
    const vector<StateID> &state_ids, int abstract_state_id, Cost solution_cost, bool split_unwanted_values) {
    compute_splits_timer.resume();
    const AbstractState &abstract_state = abstraction.get_state(abstract_state_id);

    if (log.is_at_least_debug()) {
        log << endl;
        log << "Create split for abstract state " << abstract_state_id << " and "
            << state_ids.size() << " concrete states." << endl;
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
                        for (StateID state_id : state_ids) {
                            State state = state_registry->lookup_state(state_id);
                            int state_value = state[var].get_value();
                            if (state_value != goal_value && abstract_state.contains(var, state_value)) {
                                if (log.is_at_least_debug()) {
                                    log << "add_split(var " << var << ", val " << state_value
                                        << "!=" << goal_value << ")" << endl;
                                }
                                add_split(splits, Split(
                                              abstract_state_id, var, goal_value,
                                              {state_value}, 1), true);
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

unique_ptr<Split> FlawSearch::get_single_split(const utils::CountdownTimer &cegar_timer, Cost solution_cost) {
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

        return create_split({state_id}, flawed_state.abs_id, solution_cost, split_unwanted_values);
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
FlawSearch::get_min_h_batch_split(const utils::CountdownTimer &cegar_timer, Cost solution_cost) {
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
        split = create_split(flawed_state.concrete_states, flawed_state.abs_id, solution_cost, split_unwanted_values);

        if (!utils::extra_memory_padding_is_reserved()) {
            return nullptr;
        }

        if (split) {
            last_refined_flawed_state = move(flawed_state);
        } else {
            last_refined_flawed_state = FlawedState::no_state;
            // We selected an abstract state without any flaws, so we try again.
            return get_min_h_batch_split(cegar_timer, solution_cost);
        }

        return split;
    }

    assert(search_status == SOLVED);
    return nullptr;
}

FlawSearch::FlawSearch(
    const shared_ptr<AbstractTask> &task,
    const Abstraction &abstraction,
    ShortestPaths &shortest_paths,
    shared_ptr<TransitionSystem> &simulated_transition_system,
    utils::RandomNumberGenerator &rng,
    PickFlawedAbstractState pick_flawed_abstract_state,
    PickSplit pick_split,
    FilterSplit filter_split,
    PickSplit tiebreak_split,
    PickSequenceFlaw sequence_split,
    PickSequenceFlaw sequence_tiebreak_split,
    int max_concrete_states_per_abstract_state,
    int max_state_expansions,
    bool intersect_flaw_search_abstract_states,
    lp::LPSolverType lp_solver,
    shared_ptr<disambiguation::DisambiguationMethod> flaw_search_states_disambiguation,
    const utils::LogProxy &log) :
    task_proxy(*task),
    domain_sizes(get_domain_sizes(task_proxy)),
    abstraction(abstraction),
    shortest_paths(shortest_paths),
    split_selector(task,
                   shortest_paths,
                   abstraction,
                   simulated_transition_system,
                   pick_split,
                   filter_split,
                   tiebreak_split,
                   sequence_split,
                   sequence_tiebreak_split,
                   lp_solver,
                   log.is_at_least_debug()),
    rng(rng),
    pick_flawed_abstract_state(pick_flawed_abstract_state),
    max_concrete_states_per_abstract_state(max_concrete_states_per_abstract_state),
    max_state_expansions(max_state_expansions),
    intersect_flaw_search_abstract_states(intersect_flaw_search_abstract_states),
    flaw_search_states_disambiguation(flaw_search_states_disambiguation),
    log(log),
    silent_log(utils::get_silent_log()),
    sequence_flaws_queue(),
    last_refined_flawed_state(FlawedState::no_state),
    best_flaw_h((pick_flawed_abstract_state == PickFlawedAbstractState::MAX_H) ? 0 : INF),
    splits_cache(),
    opt_tr_cache(),
    num_searches(0),
    num_overall_expanded_concrete_states(0),
    max_expanded_concrete_states(0),
    flaw_search_timer(false),
    compute_splits_timer(false),
    pick_split_timer(false) {
    // legacy_flaws, in_sequence or new flaws (default).
    switch (pick_flawed_abstract_state) {
    case PickFlawedAbstractState::FIRST_ON_SHORTEST_PATH:
    case PickFlawedAbstractState::FIRST_ON_SHORTEST_PATH_UNWANTED_VALUES:
    case PickFlawedAbstractState::FIRST_ON_SHORTEST_PATH_BACKWARD_WANTED_VALUES:
    case PickFlawedAbstractState::FIRST_ON_SHORTEST_PATH_BACKWARD_WANTED_VALUES_REFINING_INIT_STATE:
    case PickFlawedAbstractState::FIRST_ON_SHORTEST_PATH_BACKWARD:
    case PickFlawedAbstractState::FIRST_ON_SHORTEST_PATH_BIDIRECTIONAL_INTERLEAVED:
    case PickFlawedAbstractState::FIRST_ON_SHORTEST_PATH_BIDIRECTIONAL_BACKWARD_FORWARD:
    case PickFlawedAbstractState::FIRST_ON_SHORTEST_PATH_BIDIRECTIONAL_FORWARD_BACKWARD:
    case PickFlawedAbstractState::FIRST_ON_SHORTEST_PATH_BIDIRECTIONAL_CLOSEST_TO_GOAL:
        legacy_flaws = true;
        break;
    case PickFlawedAbstractState::SEQUENCE:
    case PickFlawedAbstractState::SEQUENCE_IN_ABSTRACTION:
    case PickFlawedAbstractState::SEQUENCE_BACKWARD:
    case PickFlawedAbstractState::SEQUENCE_IN_ABSTRACTION_BACKWARD:
    case PickFlawedAbstractState::SEQUENCE_BIDIRECTIONAL:
    case PickFlawedAbstractState::SEQUENCE_IN_ABSTRACTION_BIDIRECTIONAL:
    case PickFlawedAbstractState::SEQUENCE_ITERATIVE_IN_ABSTRACTION:
    case PickFlawedAbstractState::SEQUENCE_ITERATIVE_IN_ABSTRACTION_BACKWARD:
    case PickFlawedAbstractState::SEQUENCE_ITERATIVE_IN_ABSTRACTION_BIDIRECTIONAL:
    case PickFlawedAbstractState::SEQUENCE_BATCH:
    case PickFlawedAbstractState::SEQUENCE_BATCH_BACKWARD:
    case PickFlawedAbstractState::SEQUENCE_IN_ABSTRACTION_BATCH:
    case PickFlawedAbstractState::SEQUENCE_IN_ABSTRACTION_BATCH_BACKWARD:
        in_sequence = true;
        break;
    default:
        break;
    }
    // only_in_abstraction and in_batch switches.
    switch (pick_flawed_abstract_state) {
    case PickFlawedAbstractState::SEQUENCE_IN_ABSTRACTION:
    case PickFlawedAbstractState::SEQUENCE_IN_ABSTRACTION_BACKWARD:
    case PickFlawedAbstractState::SEQUENCE_IN_ABSTRACTION_BIDIRECTIONAL:
        only_in_abstraction = InAbstractionFlawSearchKind::TRUE;
        break;
    case PickFlawedAbstractState::SEQUENCE_BATCH:
    case PickFlawedAbstractState::SEQUENCE_BATCH_BACKWARD:
        in_batch = true;
        break;
    case PickFlawedAbstractState::SEQUENCE_IN_ABSTRACTION_BATCH:
    case PickFlawedAbstractState::SEQUENCE_IN_ABSTRACTION_BATCH_BACKWARD:
        only_in_abstraction = InAbstractionFlawSearchKind::TRUE;
        in_batch = true;
        break;
    case PickFlawedAbstractState::SEQUENCE_ITERATIVE_IN_ABSTRACTION:
    case PickFlawedAbstractState::SEQUENCE_ITERATIVE_IN_ABSTRACTION_BACKWARD:
    case PickFlawedAbstractState::SEQUENCE_ITERATIVE_IN_ABSTRACTION_BIDIRECTIONAL:
        only_in_abstraction = InAbstractionFlawSearchKind::ITERATIVE_IN_REGRESSION;
        break;
    default:
        break;
    }
    // Direction (FIRST_ON_SHORTEST_PATH_BIDIRECTIONAL_CLOSEST_TO_GOAL is
    // determined in its function).
    switch (pick_flawed_abstract_state) {
    case PickFlawedAbstractState::FIRST_ON_SHORTEST_PATH:
        forward_direction = true;
        break;
    case PickFlawedAbstractState::FIRST_ON_SHORTEST_PATH_UNWANTED_VALUES:
        forward_direction = true;
        split_unwanted_values = true;
        break;
    case PickFlawedAbstractState::FIRST_ON_SHORTEST_PATH_BACKWARD_WANTED_VALUES:
    case PickFlawedAbstractState::FIRST_ON_SHORTEST_PATH_BACKWARD_WANTED_VALUES_REFINING_INIT_STATE:
        backward_direction = true;
        break;
    case PickFlawedAbstractState::FIRST_ON_SHORTEST_PATH_BACKWARD:
        backward_direction = true;
        split_unwanted_values = true;
        break;
    case PickFlawedAbstractState::FIRST_ON_SHORTEST_PATH_BIDIRECTIONAL_INTERLEAVED:
    case PickFlawedAbstractState::FIRST_ON_SHORTEST_PATH_BIDIRECTIONAL_FORWARD_BACKWARD:
        forward_direction = true;
        break;
    case PickFlawedAbstractState::FIRST_ON_SHORTEST_PATH_BIDIRECTIONAL_BACKWARD_FORWARD:
        // Note that the interleaved case starts as backward but its value is
        // modified inside the refinement loop.
        backward_direction = true;
        split_unwanted_values = true;
        break;
    case PickFlawedAbstractState::SEQUENCE:
    case PickFlawedAbstractState::SEQUENCE_IN_ABSTRACTION:
    case PickFlawedAbstractState::SEQUENCE_ITERATIVE_IN_ABSTRACTION:
    case PickFlawedAbstractState::SEQUENCE_BATCH:
    case PickFlawedAbstractState::SEQUENCE_IN_ABSTRACTION_BATCH:
        forward_direction = true;
        break;
    case PickFlawedAbstractState::SEQUENCE_BACKWARD:
    case PickFlawedAbstractState::SEQUENCE_IN_ABSTRACTION_BACKWARD:
    case PickFlawedAbstractState::SEQUENCE_ITERATIVE_IN_ABSTRACTION_BACKWARD:
    case PickFlawedAbstractState::SEQUENCE_BATCH_BACKWARD:
    case PickFlawedAbstractState::SEQUENCE_IN_ABSTRACTION_BATCH_BACKWARD:
        backward_direction = true;
        break;
    case PickFlawedAbstractState::SEQUENCE_BIDIRECTIONAL:
    case PickFlawedAbstractState::SEQUENCE_IN_ABSTRACTION_BIDIRECTIONAL:
    case PickFlawedAbstractState::SEQUENCE_ITERATIVE_IN_ABSTRACTION_BIDIRECTIONAL:
        forward_direction = true;
        backward_direction = true;
        break;
    default:
        forward_direction = true;
        break;
    }
}

SplitProperties FlawSearch::get_split(const utils::CountdownTimer &cegar_timer, Cost solution_cost) {
    unique_ptr<Split> split;
    int found_flaws = 0;

    switch (pick_flawed_abstract_state) {
    case PickFlawedAbstractState::FIRST:
    case PickFlawedAbstractState::RANDOM:
    case PickFlawedAbstractState::MIN_H:
    case PickFlawedAbstractState::MAX_H:
        split = get_single_split(cegar_timer, solution_cost);
        if (split) {
            found_flaws = 1;
        }
        break;
    case PickFlawedAbstractState::BATCH_MIN_H:
        // TODO: get the number of found flaws with batch split
        split = get_min_h_batch_split(cegar_timer, solution_cost);
        if (split) {
            found_flaws = 1;
        }
        break;
    default:
        log << "Invalid pick flaw strategy: "
            << static_cast<int>(pick_flawed_abstract_state)
            << endl;
        utils::exit_with(utils::ExitCode::SEARCH_INPUT_ERROR);
    }

    if (split) {
        assert(!(pick_flawed_abstract_state == PickFlawedAbstractState::MAX_H
                 || pick_flawed_abstract_state == PickFlawedAbstractState::MIN_H)
               || best_flaw_h == get_h_value(split->abstract_state_id));
    }
    // TODO: compute the percentage of the flawed split respect the plan length.
    return SplitProperties(move(split), 0, false, found_flaws, 0);
}

SplitProperties FlawSearch::get_split_and_direction(const Solution &solution,
                                                    const utils::CountdownTimer &cegar_timer,
                                                    const bool half_limits_reached) {
    update_current_direction(half_limits_reached);
    if (pick_flawed_abstract_state == PickFlawedAbstractState::FIRST_ON_SHORTEST_PATH_BIDIRECTIONAL_CLOSEST_TO_GOAL) {
        return get_split_legacy_closest_to_goal(solution);
    } else if (in_sequence) {
        return get_sequence_splits(solution);
    } else if (legacy_flaws) {
        return get_split_legacy(solution);
    } else {
        return get_split(cegar_timer, get_optimal_plan_cost(solution, task_proxy));
    }
}

bool FlawSearch::refine_init_state() const {
    return pick_flawed_abstract_state == PickFlawedAbstractState::FIRST_ON_SHORTEST_PATH_BACKWARD_WANTED_VALUES_REFINING_INIT_STATE;
}

bool FlawSearch::refine_goals() const {
    bool refine_goals = false;
    switch (pick_flawed_abstract_state) {
    case PickFlawedAbstractState::FIRST:
    case PickFlawedAbstractState::FIRST_ON_SHORTEST_PATH:
    case PickFlawedAbstractState::FIRST_ON_SHORTEST_PATH_UNWANTED_VALUES:
    case PickFlawedAbstractState::FIRST_ON_SHORTEST_PATH_BACKWARD_WANTED_VALUES:
    case PickFlawedAbstractState::RANDOM:
    case PickFlawedAbstractState::MIN_H:
    case PickFlawedAbstractState::MAX_H:
    case PickFlawedAbstractState::BATCH_MIN_H:
        refine_goals = true;
        break;
    default:
        break;
    }
    return refine_goals;
}

void FlawSearch::update_current_direction(const bool half_limits_reached) {
    switch (pick_flawed_abstract_state) {
    case PickFlawedAbstractState::FIRST_ON_SHORTEST_PATH_BIDIRECTIONAL_INTERLEAVED:
        backward_direction = !backward_direction;
        forward_direction = !forward_direction;
        split_unwanted_values = backward_direction;
        break;
    case PickFlawedAbstractState::FIRST_ON_SHORTEST_PATH_BIDIRECTIONAL_BACKWARD_FORWARD:
    case PickFlawedAbstractState::FIRST_ON_SHORTEST_PATH_BIDIRECTIONAL_FORWARD_BACKWARD:
        if (!batch_bidirectional_already_changed_dir && half_limits_reached) {
            backward_direction = !backward_direction;
            forward_direction = !forward_direction;
            split_unwanted_values = backward_direction;
            batch_bidirectional_already_changed_dir = true;
        }
        break;
    default:
        break;
    }
}

double FlawSearch::get_plan_perc(int abstract_state_id, const Solution &solution) {
    if (abstraction.get_initial_state().get_id() == abstract_state_id) {
        return 0;
    }
    int pos = 0;
    for (const Transition &step : solution) {
        pos++;
        if (step.target_id == abstract_state_id) {
            break;
        }
    }

    return (double)pos / (double)solution.size();
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

static plugins::TypedEnumPlugin<PickFlawedAbstractState> _enum_plugin({
        {"first",
         "Consider first encountered flawed abstract state and a random concrete state."},
        {"first_on_shortest_path",
         "Follow the arbitrary solution in the shortest path tree (no flaw search). "
         "Consider first encountered flawed abstract state and a random concrete state."},
        {"first_on_shortest_path_unwanted_values",
         "Legacy code: follow the arbitrary solution in shortest path tree (no flaw search)"
         "splitting the unwanted values."
         "Consider first encountered flawed abstract state + a random concrete state."},
        {"first_on_shortest_path_backward",
         "Follow the arbitrary solution in shortest path in backward direction"
         "(from the goal) splitting the unwanted values."},
        {"first_on_shortest_path_backward_wanted_values",
         "Follow the arbitrary solution in shortest path in backward direction"
         "from the goal) splitting the wanted values."},
        {"first_on_shortest_path_backward_wanted_values_refining_init_state",
         "Follow the arbitrary solution in shortest path in backward direction"
         "(from the goal) splitting the wanted values refining the init state"
         "before refinement steps."},
        {"first_on_shortest_path_bidirectional_interleaved",
         "Follow the arbitrary solution in shortest path in backward and forward"
         "directions interleaving them."},
        {"first_on_shortest_path_bidirectional_backward_forward",
         "Follow the arbitrary solution in shortest path in backward direction the"
         "first half of the time/states/transitions and in the forward direction the other one."},
        {"first_on_shortest_path_bidirectional_forward_backward",
         "Follow the arbitrary solution in shortest path in forward direction the"
         "first half of the time/states/transitions and in the backward direction the other one."},
        {"FIRST_ON_SHORTEST_PATH_BIDIRECTIONAL_CLOSEST_TO_GOAL",
         "Follow the arbitrary solution in shortest path in the direction where"
         "the found flaw is closer to the goal."},
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
        {"sequence", "Sequence flaws in the forward direction splitting wanted values."},
        {"sequence_in_abstraction",
         "Sequence flaws in the forward direction only over the abstraction (without taking into account init state nor goals) splitting wanted values."},
        {"sequence_backward", "Sequence flaws in the backward direction splitting unwanted values."},
        {"sequence_in_abstraction_backward",
         "Sequence flaws in the backward direction only over the abstraction (without taking into account init state nor goals) splitting unwanted values."},
        {"sequence_bidirectional", "Sequence flaws in the both directions splitting wanted values in the forward direction and unwanted values in the backward direction."},
        {"sequence_in_abstraction_bidirectional",
         "Sequence flaws in the both directions only over the abstraction (without taking into account init state nor goals) splitting wanted values in the forward direction and unwanted values in the backward direction."},
        {"sequence_iterative_in_abstraction",
         "Sequence flaws in the forward direction iteratively in abstraction from the goals, starting at the initial state when no one is found."},
        {"sequence_iterative_in_abstraction_backward",
         "Sequence flaws in the backward direction iteratively in abstraction from the goals, starting at the initial state when no one is found."},
        {"sequence_iterative_in_abstraction_bidirectional",
         "Sequence flaws in both directions iteratively in abstraction from the goals, starting at the initial state when no one is found."},
        {"sequence_batch",
         "Sequence progression flaws refining all flaws before the next flaws search."},
        {"sequence_batch_backward",
         "Sequence regression flaws refining all flaws before the next flaws search."},
        {"sequence_in_abstraction_batch",
         "Sequence in abstraction progression flaws refining all flaws before the next flaws search."},
        {"sequence_in_abstraction_batch_backward",
         "Sequence in abstraction regression flaws refining all flaws before the next flaws search."},
    });
}
