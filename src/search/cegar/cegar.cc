#include "cegar.h"

#include "abstraction.h"
#include "abstract_state.h"
#include "cartesian_set.h"
#include "flaw_search.h"
#include "shortest_paths.h"
#include "transition_system.h"
#include "utils.h"

#include "../task_utils/task_properties.h"
#include "../utils/language.h"
#include "../utils/logging.h"
#include "../utils/math.h"
#include "../utils/memory.h"

#include <algorithm>
#include <cassert>
#include <iostream>
#include <unordered_map>

using namespace std;

namespace cegar {
CEGAR::CEGAR(
    const shared_ptr<AbstractTask> &task,
    int max_states,
    int max_non_looping_transitions,
    double max_time,
    PickFlawedAbstractState pick_flawed_abstract_state,
    PickSplit pick_split,
    PickSplit tiebreak_split,
    PickSplit sequence_split,
    int max_concrete_states_per_abstract_state,
    int max_state_expansions,
    bool intersect_flaw_search_abstract_states,
    utils::RandomNumberGenerator &rng,
    utils::LogProxy &log,
    DotGraphVerbosity dot_graph_verbosity)
    : task_proxy(*task),
      domain_sizes(get_domain_sizes(task_proxy)),
      max_states(max_states),
      max_non_looping_transitions(max_non_looping_transitions),
      pick_flawed_abstract_state(pick_flawed_abstract_state),
      abstraction(utils::make_unique_ptr<Abstraction>(task, log)),
      timer(max_time),
      max_time(max_time),
      log(log),
      dot_graph_verbosity(dot_graph_verbosity) {
    assert(max_states >= 1);
    shortest_paths = utils::make_unique_ptr<ShortestPaths>(
        task_properties::get_operator_costs(task_proxy), log);
    flaw_search = utils::make_unique_ptr<FlawSearch>(
        task, *abstraction, *shortest_paths, rng,
        pick_flawed_abstract_state, pick_split, tiebreak_split, sequence_split,
        max_concrete_states_per_abstract_state, max_state_expansions,
        intersect_flaw_search_abstract_states, log);

    if (log.is_at_least_normal()) {
        log << "Start building abstraction." << endl;
        log << "Time limit: " << timer.get_remaining_time() << endl;
        log << "Maximum number of states: " << max_states << endl;
        log << "Maximum number of transitions: "
            << max_non_looping_transitions << endl;
    }

    refinement_loop();
    if (log.is_at_least_normal()) {
        log << "Done building abstraction." << endl;
        log << "Time for building abstraction: " << timer.get_elapsed_time() << endl;
        print_statistics();
    }
}

CEGAR::~CEGAR() {
}

unique_ptr<Abstraction> CEGAR::extract_abstraction() {
    assert(abstraction);
    return move(abstraction);
}

void CEGAR::separate_facts_unreachable_before_goal(bool refine_goals) const {
    assert(abstraction->get_goals().size() == 1);
    assert(abstraction->get_num_states() == 1);
    assert(task_proxy.get_goals().size() == 1);
    FactProxy goal = task_proxy.get_goals()[0];
    utils::HashSet<FactProxy> reachable_facts = get_relaxed_possible_before(
        task_proxy, goal);
    for (VariableProxy var : task_proxy.get_variables()) {
        if (!may_keep_refining())
            break;
        int var_id = var.get_id();
        vector<int> unreachable_values;
        for (int value = 0; value < var.get_domain_size(); ++value) {
            FactProxy fact = var.get_fact(value);
            if (reachable_facts.count(fact) == 0)
                unreachable_values.push_back(value);
        }
        if (!unreachable_values.empty())
            abstraction->refine(abstraction->get_initial_state(), var_id, unreachable_values);
    }
    abstraction->mark_all_states_as_goals();
    /*
      Split off the goal fact from the initial state. Then the new initial
      state is the only non-goal state and no goal state will have to be split
      later.

      For all states s in which the landmark might have been achieved we need
      h(s)=0. If the limits don't allow splitting off all facts unreachable
      before the goal to achieve this, we instead preserve h(s)=0 for *all*
      states s and cannot split off the goal fact from the abstract initial
      state.
    */
    assert(abstraction->get_initial_state().includes(task_proxy.get_initial_state()));
    assert(reachable_facts.count(goal));
    if (refine_goals && may_keep_refining()) {
        abstraction->refine(abstraction->get_initial_state(), goal.get_variable().get_id(), {goal.get_value()});
    }
}

bool CEGAR::may_keep_refining(bool in_current_direction) const {
    int divider = 1;
    if (in_current_direction) {
        divider = 2;
    }
    string half_of = "";
    if (in_current_direction) {
        half_of = "the half of ";
    }
    if (abstraction->get_num_states() >= max_states / divider) {
        if (log.is_at_least_normal()) {
            log << "Reached " << half_of << "maximum number of states." << endl;
        }
        return false;
    } else if (abstraction->get_transition_system().get_num_non_loops() >= max_non_looping_transitions / divider) {
        if (log.is_at_least_normal()) {
            log << "Reached " << half_of << "maximum number of transitions." << endl;
        }
        return false;
    } else if (max_time != numeric_limits<double>::infinity() && timer.get_elapsed_time() >= max_time / double(divider)) {
        if (log.is_at_least_normal()) {
            log << "Reached " << half_of << "time limit." << endl;
        }
        return false;
    } else if (!utils::extra_memory_padding_is_reserved()) {
        if (log.is_at_least_normal()) {
            log << "Reached " << half_of << "memory limit." << endl;
        }
        return false;
    }
    return true;
}

void CEGAR::refinement_loop() {
    int forward_refinements = 0;
    int backward_refinements = 0;
    int forward_flaws = 0;
    int backward_flaws = 0;
    /*
      For landmark tasks we have to map all states in which the
      landmark might have been achieved to arbitrary abstract goal
      states. For the other types of subtasks our method won't find
      unreachable facts, but calling it unconditionally for subtasks
      with one goal doesn't hurt and simplifies the implementation.

      In any case, we separate all goal states from non-goal states
      to simplify the implementation. This way, we don't have to split
      goal states later.
    */
    bool refine_goals = flaw_search->refine_goals();
    if (task_proxy.get_goals().size() == 1) {
        separate_facts_unreachable_before_goal(refine_goals);
    } else if (refine_goals) {
        // Iteratively split off the next goal fact from the current goal state.
        assert(abstraction->get_num_states() == 1);
        const AbstractState *current = &abstraction->get_initial_state();
        for (FactProxy goal : task_proxy.get_goals()) {
            if (!may_keep_refining()) {
                break;
            }
            FactPair fact = goal.get_pair();
            auto pair = abstraction->refine(*current, fact.var, {fact.value});
            current = &abstraction->get_state(pair.second);
        }
        assert(!abstraction->get_goals().count(abstraction->get_initial_state().get_id()));
        assert(abstraction->get_goals().size() == 1);
    }
    if (flaw_search->refine_init_state()) {
        // Split iteratively until the abstract initial state is exactly
        // the concrete initial state, as done with goals in forward direction
        // because the refinement functions only work with optimal transitions and
        // the initial abstract state has not any (because it is the state with
        // distance to init=0 and all operators have cost>=0).
        for (FactProxy init_value : task_proxy.get_initial_state()) {
            if (!may_keep_refining()) {
                break;
            }
            FactPair fact = init_value.get_pair();
            // The other values are split from the initial state
            VariableProxy var = task_proxy.get_variables()[fact.var];
            vector<int> other_values{};
            for (int i = 0; i < var.get_domain_size(); i++) {
                int var_value = var.get_fact(i).get_value();
                if (var_value != fact.value &&
                    abstraction->get_initial_state().contains(fact.var, var_value)) {
                    other_values.push_back(var_value);
                }
            }
            if (!other_values.empty()) {
                abstraction->refine(abstraction->get_initial_state(), fact.var, other_values);
            }
        }
    }


    // Initialize abstract goal distances and shortest path tree.
    shortest_paths->recompute(
        abstraction->get_transition_system().get_incoming_transitions(),
        abstraction->get_transition_system().get_outgoing_transitions(),
        abstraction->get_goals(),
        abstraction->get_initial_state().get_id());
    assert(shortest_paths->test_distances(
               abstraction->get_transition_system().get_incoming_transitions(),
               abstraction->get_transition_system().get_outgoing_transitions(),
               abstraction->get_goals()));

    utils::Timer find_trace_timer(false);
    utils::Timer find_flaw_timer(false);
    utils::Timer refine_timer(false);
    utils::Timer update_goal_distances_timer(false);

    int n_refinements = abstraction->get_num_states() - 1;
    bool half_limits_reached = false;
    while (may_keep_refining()) {
        find_trace_timer.resume();
        unique_ptr<Solution> solution;
        solution = shortest_paths->extract_solution(
            abstraction->get_initial_state().get_id(), abstraction->get_goals());
        find_trace_timer.stop();

        if (solution) {
            int new_abstract_solution_cost =
                shortest_paths->get_32bit_goal_distance(abstraction->get_initial_state().get_id());
            if (new_abstract_solution_cost > old_abstract_solution_cost) {
                old_abstract_solution_cost = new_abstract_solution_cost;
                if (log.is_at_least_normal()) {
                    log << "Abstract solution cost: " << old_abstract_solution_cost << endl;
                }
            }
        } else {
            log << "Abstract task is unsolvable." << endl;
            break;
        }

        find_flaw_timer.resume();

        // Dump/write dot file for current abstraction.
        if (dot_graph_verbosity == DotGraphVerbosity::WRITE_TO_CONSOLE) {
            cout << create_dot_graph(task_proxy, *abstraction) << endl;
        } else if (dot_graph_verbosity == DotGraphVerbosity::WRITE_TO_FILE) {
            write_to_file(
                "graph" + to_string(abstraction->get_num_states()) + ".dot",
                create_dot_graph(task_proxy, *abstraction));
        } else if (dot_graph_verbosity != DotGraphVerbosity::SILENT) {
            ABORT("Invalid dot graph verbosity");
        }

        // Once reached, it has not to be checked anymore.
        if (!half_limits_reached) {
            half_limits_reached = !may_keep_refining(true);
        }
        SplitProperties split_prop =
            flaw_search->get_split_and_direction(*solution,
                                                 timer,
                                                 half_limits_reached);
        find_flaw_timer.stop();

        if (!utils::extra_memory_padding_is_reserved()) {
            log << "Reached memory limit in flaw search." << endl;
            break;
        }

        if (timer.is_expired()) {
            log << "Reached time limit in flaw search." << endl;
            break;
        }

        if (!split_prop.split) {
            log << "Found concrete solution." << endl;
            break;
        }

        refine_timer.resume();
        int state_id = split_prop.split->abstract_state_id;
        const AbstractState &abstract_state = abstraction->get_state(state_id);
        // This may not happen in the backward direction.
        // assert(!abstraction->get_goals().count(state_id));

        pair<int, int> new_state_ids = abstraction->refine(
            abstract_state, split_prop.split->var_id, split_prop.split->values);
        refine_timer.stop();

        n_refinements++;
        if (split_prop.backward_direction) {
            backward_refinements++;
        } else {
            forward_refinements++;
        }
        forward_flaws += split_prop.n_forward_flaws;
        backward_flaws += split_prop.n_backward_flaws;
        if (n_refinements % 100 == 0) {
            if (log.is_at_least_normal()) {
                log << "Number of refinements: " << n_refinements << endl;
                log << "Forward refinements: " << forward_refinements << endl;
                log << "Backward refinements: " << backward_refinements << endl;
                log << "Total forward flaws found: " << forward_flaws << endl;
                log << "Total backward flaws found: " << backward_flaws << endl;
            }
        }

        update_goal_distances_timer.resume();
        shortest_paths->update_incrementally(
            abstraction->get_transition_system().get_incoming_transitions(),
            abstraction->get_transition_system().get_outgoing_transitions(),
            state_id, new_state_ids.first, new_state_ids.second,
            abstraction->get_goals(),
            abstraction->get_initial_state().get_id());
        assert(shortest_paths->test_distances(
                   abstraction->get_transition_system().get_incoming_transitions(),
                   abstraction->get_transition_system().get_outgoing_transitions(),
                   abstraction->get_goals()));
        update_goal_distances_timer.stop();

        if (log.is_at_least_verbose() &&
            abstraction->get_num_states() % 1000 == 0) {
            log << abstraction->get_num_states() << "/" << max_states << " states, "
                << abstraction->get_transition_system().get_num_non_loops() << "/"
                << max_non_looping_transitions << " transitions" << endl;
        }
    }
    if (log.is_at_least_normal()) {
        log << "Time for finding abstract traces: " << find_trace_timer << endl;
        log << "Time for finding flaws and computing splits: " << find_flaw_timer << endl;
        log << "Time for splitting states: " << refine_timer << endl;
        log << "Time for updating goal distances: " << update_goal_distances_timer << endl;
        log << "Number of refinements: " << n_refinements << endl;
        log << "Forward refinements: " << forward_refinements << endl;
        log << "Backward refinements: " << backward_refinements << endl;
        log << "Total forward flaws found: " << forward_flaws << endl;
        log << "Total backward flaws found: " << backward_flaws << endl;
    }
}

void CEGAR::print_statistics() const {
    abstraction->print_statistics();
    flaw_search->print_statistics();
}
}
