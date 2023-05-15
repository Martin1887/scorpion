#include "symbolic_comparing_cost_saturation.h"

#include "../cegar/abstract_search.h"
#include "../cegar/abstract_state.h"
#include "../cegar/abstraction.h"
#include "../cegar/cartesian_heuristic_function.h"
#include "../cegar/cegar.h"
#include "../cegar/cost_saturation.h"
#include "../cegar/subtask_generators.h"
#include "../cegar/transition_system.h"

#include "../task_utils/task_properties.h"
#include "symbolic_uniform_backwards_search_heuristic.h"
#include <memory>

using namespace cegar;
using namespace std;

namespace cegar_symbolic_comparison {
CegarSymbolicComparingCostSaturation::CegarSymbolicComparingCostSaturation(
    const vector < shared_ptr < SubtaskGenerator >> &subtask_generators,
    int max_states,
    int max_non_looping_transitions,
    double max_time,
    bool use_general_costs,
    PickFlawedAbstractState pick_flawed_abstract_state,
    PickSplit pick_split,
    PickSplit tiebreak_split,
    int max_concrete_states_per_abstract_state,
    int max_state_expansions,
    SearchStrategy search_strategy,
    int memory_padding_mb,
    utils::RandomNumberGenerator &rng,
    utils::LogProxy &log,
    DotGraphVerbosity dot_graph_verbosity,
    const Options &opts,
    shared_ptr < symbolic::SymVariables > vars)
    :
      CostSaturation(
          subtask_generators,
          max_states,
          max_non_looping_transitions,
          max_time,
          use_general_costs,
          pick_flawed_abstract_state,
          pick_split,
          tiebreak_split,
          max_concrete_states_per_abstract_state,
          max_state_expansions,
          search_strategy,
          memory_padding_mb,
          rng,
          log,
          dot_graph_verbosity),
      opts(opts),
      vars(vars) {
}

void CegarSymbolicComparingCostSaturation::build_abstractions(
    const vector < shared_ptr < AbstractTask >> &subtasks,
    const utils::CountdownTimer &timer,
    const function < bool() > &should_abort) {
    int rem_subtasks = subtasks.size();
    int subtasks_factor = 2;
    // The remaining time is divided by 2 (symbolic and CEGAR) when there is
    // only a task and by 4 when there are more subtasks because the
    // post-reducing-costs symbolic and CEGAR executions
    if (subtasks.size() > 1) {
        subtasks_factor = 4;
    }

    for (shared_ptr < AbstractTask > subtask : subtasks) {
        subtask = get_remaining_costs_task(subtask);
        assert(num_states < max_states);

        TaskProxy subtask_proxy(*subtask);

        // call the symbolic uniform cost back search on this subtask and
        // print the h value in the initial state
        double t0 = timer.get_elapsed_time();
        SymUniformBackSearchHeuristic sym(opts, vars, timer.get_remaining_time() / rem_subtasks / subtasks_factor, subtask);
        double t1 = timer.get_elapsed_time();
        log << "Symbolic time duration: " << (t1 - t0) << endl << endl;
        int h0 = sym.h_value(subtask_proxy.get_initial_state());
        log << "Symbolic initial h value: " << h0 << endl << endl;

        double t2 = timer.get_elapsed_time();
        CEGAR cegar(
            subtask,
            max(1, (max_states - num_states) / rem_subtasks),
            max(1, (max_non_looping_transitions - num_non_looping_transitions) /
                rem_subtasks),
            timer.get_remaining_time() / rem_subtasks / subtasks_factor,
            pick_flawed_abstract_state,
            pick_split,
            tiebreak_split,
            max_concrete_states_per_abstract_state,
            max_state_expansions,
            search_strategy,
            rng,
            log,
            dot_graph_verbosity);
        double t3 = timer.get_elapsed_time();
        log << "CEGAR time duration: " << (t3 - t2) << endl << endl;

        unique_ptr < Abstraction > abstraction = cegar.extract_abstraction();
        num_states += abstraction->get_num_states();
        num_non_looping_transitions += abstraction->get_transition_system().get_num_non_loops();
        assert(num_states <= max_states);

        vector < int > costs = task_properties::get_operator_costs(TaskProxy(*subtask));
        vector < int > init_distances = compute_distances(
            abstraction->get_transition_system().get_outgoing_transitions(),
            costs,
            {abstraction->get_initial_state().get_id()});
        vector < int > goal_distances = compute_distances(
            abstraction->get_transition_system().get_incoming_transitions(),
            costs,
            abstraction->get_goals());

        int num_unsolvable_states = count(goal_distances.begin(), goal_distances.end(), INF);
        log << "Unsolvable Cartesian states: " << num_unsolvable_states << endl;
        log << "CEGAR initial h value: "
            << goal_distances[abstraction->get_initial_state().get_id()]
            << endl << endl;

        vector < int > saturated_costs = compute_saturated_costs(
            abstraction->get_transition_system(),
            init_distances,
            goal_distances,
            use_general_costs);

        reduce_remaining_costs(saturated_costs);

        // call CEGAR and the symbolic uniform cost back search on this subtask and
        // print the h value in the initial state only if additive (more than 1 subtasks)
        if (subtasks.size() > 1) {
            shared_ptr<AbstractTask> remaining_costs_subtask = get_remaining_costs_task(subtask);
            double t4 = timer.get_elapsed_time();
            SymUniformBackSearchHeuristic sym_post(opts, vars, timer.get_remaining_time() / rem_subtasks / subtasks_factor, remaining_costs_subtask);
            double t5 = timer.get_elapsed_time();
            log << "After costs reduction symbolic time duration: " << (t5 - t4) << endl << endl;
            int h0_post = sym_post.h_value(subtask_proxy.get_initial_state());
            log << "After costs reduction symbolic initial h value: " << h0_post << endl << endl;

            double t6 = timer.get_elapsed_time();
            CEGAR cegar_post(
                remaining_costs_subtask,
                max(1, (max_states - num_states) / rem_subtasks),
                max(1, (max_non_looping_transitions - num_non_looping_transitions) /
                    rem_subtasks),
                timer.get_remaining_time() / rem_subtasks / subtasks_factor,
                pick_flawed_abstract_state,
                pick_split,
                tiebreak_split,
                max_concrete_states_per_abstract_state,
                max_state_expansions,
                search_strategy,
                rng,
                log,
                dot_graph_verbosity);
            double t7 = timer.get_elapsed_time();
            log << "After costs reduction CEGAR time duration: " << (t7 - t6) << endl << endl;

            unique_ptr < Abstraction > abstraction_post = cegar_post.extract_abstraction();
            vector < int > costs_post = task_properties::get_operator_costs(TaskProxy(*remaining_costs_subtask));
            vector < int > init_distances_post = compute_distances(
                abstraction_post->get_transition_system().get_outgoing_transitions(),
                costs_post,
                {abstraction_post->get_initial_state().get_id()});
            vector < int > goal_distances_post = compute_distances(
                abstraction_post->get_transition_system().get_incoming_transitions(),
                costs_post,
                abstraction_post->get_goals());

            int num_unsolvable_states_post = count(goal_distances_post.begin(), goal_distances_post.end(), INF);
            log << "After costs reduction nsolvable Cartesian states: " << num_unsolvable_states_post << endl;
            log << "After costs reduction CEGAR initial h value: "
                << goal_distances_post[abstraction_post->get_initial_state().get_id()]
                << endl << endl;
        }


        heuristic_functions.emplace_back(
            abstraction->extract_refinement_hierarchy(),
            move(goal_distances));
        --rem_subtasks;

        if (should_abort()) {
            break;
        }
    }
}
}
