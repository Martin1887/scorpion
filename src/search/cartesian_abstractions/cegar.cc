#include "cegar.h"

#include "abstraction.h"
#include "abstract_state.h"
#include "flaw_search.h"
#include "shortest_paths.h"
#include "transition_system.h"
#include "utils.h"

#include "../task_utils/cartesian_set.h"
#include "../task_utils/disambiguated_operator.h"
#include "../task_utils/disambiguation_method.h"
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
using namespace disambiguation;

namespace cartesian_abstractions {
CEGAR::CEGAR(
    const shared_ptr<AbstractTask> &task,
    int max_states,
    int max_non_looping_transitions,
    double max_time,
    PickFlawedAbstractState pick_flawed_abstract_state,
    PickSplit pick_split,
    FilterSplit filter_split,
    PickSplit tiebreak_split,
    PickSequenceFlaw sequence_split,
    PickSequenceFlaw sequence_tiebreak_split,
    int max_concrete_states_per_abstract_state,
    int max_state_expansions,
    bool intersect_flaw_search_abstract_states,
    bool refine_init,
    lp::LPSolverType lp_solver,
    shared_ptr<disambiguation::DisambiguationMethod> &operators_disambiguation,
    shared_ptr<disambiguation::DisambiguationMethod> &abstract_space_disambiguation,
    shared_ptr<disambiguation::DisambiguationMethod> &flaw_search_states_disambiguation,
    utils::RandomNumberGenerator &rng,
    utils::LogProxy &log,
    DotGraphVerbosity dot_graph_verbosity)
    : task_proxy(*task),
      domain_sizes(get_domain_sizes(task_proxy)),
      max_states(max_states),
      max_non_looping_transitions(max_non_looping_transitions),
      pick_flawed_abstract_state(pick_flawed_abstract_state),
      refine_init(refine_init),
      mutex_information(make_shared<MutexInformation>(task->mutex_information())),
      operators_disambiguation(operators_disambiguation),
      abstract_space_disambiguation(abstract_space_disambiguation),
      flaw_search_states_disambiguation(flaw_search_states_disambiguation),
      simulated_transition_system(make_shared<TransitionSystem>(operators)),
      timer(max_time),
      max_time(max_time),
      log(log),
      dot_graph_verbosity(dot_graph_verbosity) {
    assert(max_states >= 1);
    OperatorsProxy orig_ops = task_proxy.get_operators();
    operators = make_shared<vector<DisambiguatedOperator>>();
    operators->reserve(orig_ops.size());
    for (OperatorProxy op : orig_ops) {
        operators->push_back(DisambiguatedOperator(task_proxy, op, operators_disambiguation, mutex_information));
    }
    abstraction = make_unique<Abstraction>(task, operators, mutex_information, abstract_space_disambiguation, log);
    shortest_paths = make_unique<ShortestPaths>(
        task_properties::get_operator_costs(task_proxy), log);
    flaw_search = make_unique<FlawSearch>(
        task, *abstraction, *shortest_paths, simulated_transition_system, rng,
        pick_flawed_abstract_state, pick_split, filter_split, tiebreak_split,
        sequence_split, sequence_tiebreak_split,
        max_concrete_states_per_abstract_state, max_state_expansions,
        intersect_flaw_search_abstract_states, lp_solver, flaw_search_states_disambiguation, log);

    if (log.is_at_least_normal()) {
        log << "Start building abstraction." << endl;
        log << "Maximum number of states: " << max_states << endl;
        log << "Maximum number of transitions: "
            << max_non_looping_transitions << endl;
        log << "Maximum time: " << timer.get_remaining_time() << endl;
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
    vector<utils::HashSet<int>> reachable_facts = get_relaxed_possible_before(
        operators, task_proxy, goal);
    for (VariableProxy var : task_proxy.get_variables()) {
        if (!may_keep_refining())
            break;
        const CartesianSet &init_set = abstraction->get_initial_state().get_cartesian_set();
        int var_id = var.get_id();
        vector<int> unreachable_values;
        for (int value = 0; value < var.get_domain_size(); ++value) {
            FactProxy fact = var.get_fact(value);
            if (reachable_facts[var_id].count(fact.get_value()) == 0 && init_set.test(var_id, value))
                unreachable_values.push_back(value);
        }
        if (!unreachable_values.empty() &&
            init_set.count(var_id) > static_cast<int>(unreachable_values.size())) {
            abstraction->refine(abstraction->get_initial_state(), var_id, unreachable_values);
        }
    }
    abstraction->mark_all_goal_states_as_goals();
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
    assert(reachable_facts[goal.get_pair().var].count(goal.get_pair().value));
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
    int stats_iters = 1000;
    int delta_forward_refinements = 0;
    int forward_refinements = 0;
    int backward_refinements = 0;
    int forward_flawed_states = 0;
    int backward_flawed_states = 0;
    double forward_flawed_states_plan_length_perone = 0;
    double backward_flawed_states_plan_length_perone = 0;
    double forward_flawed_state_pos_plan_length_perc = 0;
    double backward_flawed_state_pos_plan_length_perc = 0;
    Cost previous_optimal_cost = 0;
    int n_optimal_cost_increased = 0;
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
            if (current->get_cartesian_set().count(fact.var) > 1) {
                auto pair = abstraction->refine(*current, fact.var, {fact.value});
                current = &abstraction->get_state(get<1>(pair));
            }
        }
        assert(!abstraction->get_goals().count(abstraction->get_initial_state().get_id()));
        assert(abstraction->get_goals().size() == 1);
    }
    if (refine_init) {
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
                    abstraction->get_initial_state().includes(fact.var, var_value)) {
                    other_values.push_back(var_value);
                }
            }
            // The state could be disambiguated, we need to check that it
            // contains any other value.
            if (!other_values.empty() &&
                abstraction->get_initial_state().get_cartesian_set().count(fact.var) > static_cast<int>(other_values.size())) {
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

        tuple<int, int, bool, Transitions, Transitions> refinement = abstraction->refine(
            abstract_state, split_prop.split->var_id, split_prop.split->values);
        refine_timer.stop();

        n_refinements++;
        if (split_prop.backward_direction) {
            backward_refinements++;
        } else {
            forward_refinements++;
            delta_forward_refinements++;
        }
        forward_flawed_states += split_prop.n_forward_flawed_states;
        backward_flawed_states += split_prop.n_backward_flawed_states;
        if (solution->size() > 0) {
            forward_flawed_states_plan_length_perone += (double)split_prop.n_forward_flawed_states / (double)solution->size();
            backward_flawed_states_plan_length_perone += (double)split_prop.n_backward_flawed_states / (double)solution->size();
        } else {
            forward_flawed_states_plan_length_perone += 1;
            backward_flawed_states_plan_length_perone += 1;
        }
        if (split_prop.backward_direction) {
            backward_flawed_state_pos_plan_length_perc += split_prop.flawed_state_pos_plan_length_perc;
        } else {
            forward_flawed_state_pos_plan_length_perc += split_prop.flawed_state_pos_plan_length_perc;
        }
        Cost optimal_cost = get_optimal_plan_cost(*solution, task_proxy);
        if (optimal_cost > previous_optimal_cost) {
            n_optimal_cost_increased++;
        }
        previous_optimal_cost = optimal_cost;
        if (n_refinements % stats_iters == 0) {
            if (log.is_at_least_normal()) {
                log << "Number of refinements: " << n_refinements << endl;
                log << "Forward refinements: " << forward_refinements << endl;
                log << "Backward refinements: " << backward_refinements << endl;
                log << "Total forward flawed states found: " << forward_flawed_states << endl;
                log << "Total backward flawed states found: " << backward_flawed_states << endl;
                log << "Average percentage of forward flawed states found in the last stats iter respect to plan length: "
                    << (100 * forward_flawed_states_plan_length_perone / stats_iters) << endl;
                log << "Average percentage of backward flawed states found in the last stats iter respect to plan length: "
                    << (100 * backward_flawed_states_plan_length_perone / stats_iters) << endl;
                double avg_fw_perc = 0;
                double avg_bw_perc = 0;
                if (delta_forward_refinements > 0) {
                    avg_fw_perc = forward_flawed_state_pos_plan_length_perc / delta_forward_refinements;
                }
                if (delta_forward_refinements < stats_iters) {
                    avg_bw_perc = backward_flawed_state_pos_plan_length_perc / (stats_iters - delta_forward_refinements);
                }
                log << "Average position of refined forward flawed states in the last stats iter respect to plan length: "
                    << avg_fw_perc << endl;
                log << "Average position of refined backward flawed states in the last stats iter respect to plan length: "
                    << avg_bw_perc << endl;
                log << "Total number of times the cost of the optimal plan has been increased: "
                    << n_optimal_cost_increased << endl;
                forward_flawed_states_plan_length_perone = 0;
                backward_flawed_states_plan_length_perone = 0;
                forward_flawed_state_pos_plan_length_perc = 0;
                backward_flawed_state_pos_plan_length_perc = 0;
                delta_forward_refinements = 0;
            }
        }

        update_goal_distances_timer.resume();
        shortest_paths->update_incrementally(
            abstraction->get_transition_system().get_incoming_transitions(),
            abstraction->get_transition_system().get_outgoing_transitions(),
            state_id, get<0>(refinement), get<1>(refinement), get<2>(refinement),
            get<3>(refinement), get<4>(refinement),
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
        log << "Total forward flawed states found: " << forward_flawed_states << endl;
        log << "Total backward flawed states found: " << backward_flawed_states << endl;
        log << "Average percentage of forward flawed states found in the last stats iter respect to plan length: "
            << (100 * forward_flawed_states_plan_length_perone / stats_iters) << endl;
        log << "Average percentage of backward flawed states found in the last stats iter respect to plan length: "
            << (100 * backward_flawed_states_plan_length_perone / stats_iters) << endl;
        double avg_fw_perc = 0;
        double avg_bw_perc = 0;
        if (delta_forward_refinements > 0) {
            avg_fw_perc = forward_flawed_state_pos_plan_length_perc / delta_forward_refinements;
        }
        if (delta_forward_refinements < stats_iters) {
            avg_bw_perc = backward_flawed_state_pos_plan_length_perc / (stats_iters - delta_forward_refinements);
        }
        log << "Average position of refined forward flawed states in the last stats iter respect to plan length: "
            << avg_fw_perc << endl;
        log << "Average position of refined backward flawed states in the last stats iter respect to plan length: "
            << avg_bw_perc << endl;
        log << "Total number of times the cost of the optimal plan has been increased: "
            << n_optimal_cost_increased << endl;
    }
}

Cost get_optimal_plan_cost(const Solution &solution, TaskProxy task_proxy) {
    Cost cost = 0;
    for (Transition trans : solution) {
        OperatorProxy op = task_proxy.get_operators()[trans.op_id];
        cost += op.get_cost();
    }

    return cost;
}

void CEGAR::print_statistics() const {
    abstraction->print_statistics();
    flaw_search->print_statistics();
}

void CEGAR::print_useless_refinements(const RefinementHierarchy &hier, const vector<int> &goal_distances) const {
    log << "Useless refinements: " << hier.n_useless_refinements(goal_distances) << endl;
}
}
