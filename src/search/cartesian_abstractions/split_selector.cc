#include "split_selector.h"

#include "abstract_state.h"
#include "abstraction.h"
#include "cegar.h"
#include "flaw.h"
#include "flaw_search.h"
#include "subtask_generators.h"
#include "utils.h"

#include "../heuristics/additive_heuristic.h"
#include "../lp/lp_solver.h"
#include "../potentials/potential_optimizer.h"
#include "../plugins/plugin.h"
#include "../utils/logging.h"
#include "../utils/memory.h"
#include "../utils/rng.h"

#include <cassert>
#include <iostream>
#include <limits>

using namespace std;

namespace cartesian_abstractions {
static vector<int> invert_vector(vector<int> source) {
    // It is assumed that the values of the vector are from 0 to n-1.
    vector<int> inverted = vector<int>(source.size());
    int i = 0;
    for (int val : source) {
        inverted[val] = i;
        i++;
    }

    return inverted;
}

bool Split::combine_with(Split &&other) {
    assert(var_id == other.var_id);
    if (*this == other) {
        return true;
    }
    // Try to switch the order to enable merging the splits.
    if (values.size() == 1 && values[0] == other.value) {
        swap(value, values[0]);
        assert(value == other.value);
    } else if (other.values.size() == 1 && value == other.values[0]) {
        swap(other.value, other.values[0]);
        assert(value == other.value);
    } else if (values.size() == 1 && other.values.size() == 1 && values[0] == other.values[0]) {
        swap(value, values[0]);
        swap(other.value, other.values[0]);
        assert(value == other.value);
    }

    if (value == other.value) {
        assert(utils::is_sorted_unique(values));
        assert(utils::is_sorted_unique(other.values));
        vector<int> combined_values;
        set_union(values.begin(), values.end(),
                  other.values.begin(), other.values.end(),
                  back_inserter(combined_values));
        swap(values, combined_values);
        return true;
    } else {
        // For now, we only combine splits that have a common singleton value.
        return false;
    }
}

PickSplit sequence_to_split(const PickSequenceFlaw pick) {
    switch (pick) {
    case PickSequenceFlaw::RANDOM:
        return PickSplit::RANDOM;
    case PickSequenceFlaw::MIN_UNWANTED:
        return PickSplit::MIN_UNWANTED;
    case PickSequenceFlaw::MAX_UNWANTED:
        return PickSplit::MAX_UNWANTED;
    case PickSequenceFlaw::MIN_REFINED:
        return PickSplit::MIN_REFINED;
    case PickSequenceFlaw::MAX_REFINED:
        return PickSplit::MAX_REFINED;
    case PickSequenceFlaw::MIN_HADD:
        return PickSplit::MIN_HADD;
    case PickSequenceFlaw::MAX_HADD:
        return PickSplit::MAX_HADD;
    case PickSequenceFlaw::MIN_CG:
        return PickSplit::MIN_CG;
    case PickSequenceFlaw::MAX_CG:
        return PickSplit::MAX_CG;
    case PickSequenceFlaw::HIGHEST_COST_OPERATOR:
        return PickSplit::HIGHEST_COST_OPERATOR;
    case PickSequenceFlaw::LOWEST_COST_OPERATOR:
        return PickSplit::LOWEST_COST_OPERATOR;
    case PickSequenceFlaw::RANDOM_VARS_ORDER:
        return PickSplit::RANDOM_VARS_ORDER;
    case PickSequenceFlaw::LANDMARKS_VARS_ORDER_HADD_DOWN:
        return PickSplit::LANDMARKS_VARS_ORDER_HADD_DOWN;
    case PickSequenceFlaw::MAX_POTENTIAL_VARS_ORDER:
        return PickSplit::MAX_POTENTIAL_VARS_ORDER;
    case PickSequenceFlaw::MIN_POTENTIAL_VARS_ORDER:
        return PickSplit::MIN_POTENTIAL_VARS_ORDER;
    case PickSequenceFlaw::LANDMARKS_VARS_ORDER_HADD_UP:
        return PickSplit::LANDMARKS_VARS_ORDER_HADD_UP;
    case PickSequenceFlaw::GOAL_DISTANCE_INCREASED:
        return PickSplit::GOAL_DISTANCE_INCREASED;
    case PickSequenceFlaw::OPTIMAL_PLAN_COST_INCREASED:
        return PickSplit::OPTIMAL_PLAN_COST_INCREASED;
    case PickSequenceFlaw::BALANCE_REFINED_CLOSEST_GOAL:
        return PickSplit::BALANCE_REFINED_CLOSEST_GOAL;
    default:
        cerr << "Invalid pick strategy for PickSplit conversion: "
             << static_cast<int>(pick) << endl;
        utils::exit_with(utils::ExitCode::SEARCH_INPUT_ERROR);
    }
}

SplitSelector::SplitSelector(
    const shared_ptr<AbstractTask> &task,
    ShortestPaths &shortest_paths,
    const Abstraction &abstraction,
    shared_ptr<TransitionSystem> &simulated_transition_system,
    PickSplit pick,
    PickSplit tiebreak_pick,
    PickSequenceFlaw sequence_pick,
    PickSequenceFlaw sequence_tiebreak_pick,
    lp::LPSolverType lp_solver,
    bool debug)
    : task(task),
      task_proxy(*task),
      shortest_paths(shortest_paths),
      abstraction(abstraction),
      simulated_transition_system(simulated_transition_system),
      debug(debug),
      first_pick(pick),
      tiebreak_pick(tiebreak_pick),
      sequence_pick(sequence_pick),
      sequence_tiebreak_pick(sequence_tiebreak_pick) {
    if (first_pick == PickSplit::MIN_HADD || first_pick == PickSplit::MAX_HADD ||
        tiebreak_pick == PickSplit::MIN_HADD || tiebreak_pick == PickSplit::MAX_HADD ||
        sequence_pick == PickSequenceFlaw::MIN_HADD || sequence_pick == PickSequenceFlaw::MAX_HADD ||
        sequence_tiebreak_pick == PickSequenceFlaw::MIN_HADD ||
        sequence_tiebreak_pick == PickSequenceFlaw::MAX_HADD) {
        additive_heuristic = create_additive_heuristic(task);
        additive_heuristic->compute_heuristic_for_cegar(
            task_proxy.get_initial_state());
    }
    if (first_pick == PickSplit::RANDOM_VARS_ORDER || tiebreak_pick == PickSplit::RANDOM_VARS_ORDER ||
        sequence_pick == PickSequenceFlaw::RANDOM_VARS_ORDER ||
        sequence_tiebreak_pick == PickSequenceFlaw::RANDOM_VARS_ORDER) {
        vector<int> sortered_vars = vector<int>(task->get_num_variables());
        // Fill vars_order with numbers from 0 to num_variables -1 to shuffle it aftwerards.
        std::iota(std::begin(sortered_vars), std::end(sortered_vars), 0);
        // This uses /dev/urandom en Linux, so the numbers are different at each execution.
        std::random_device rd;
        std::mt19937 g(rd());
        std::shuffle(sortered_vars.begin(), sortered_vars.end(), g);
        vars_order = invert_vector(sortered_vars);
    }
    if (first_pick == PickSplit::LANDMARKS_VARS_ORDER_HADD_DOWN || tiebreak_pick == PickSplit::LANDMARKS_VARS_ORDER_HADD_DOWN ||
        sequence_pick == PickSequenceFlaw::LANDMARKS_VARS_ORDER_HADD_DOWN ||
        sequence_tiebreak_pick == PickSequenceFlaw::LANDMARKS_VARS_ORDER_HADD_DOWN ||
        first_pick == PickSplit::LANDMARKS_VARS_ORDER_HADD_UP || tiebreak_pick == PickSplit::LANDMARKS_VARS_ORDER_HADD_UP ||
        sequence_pick == PickSequenceFlaw::LANDMARKS_VARS_ORDER_HADD_UP ||
        sequence_tiebreak_pick == PickSequenceFlaw::LANDMARKS_VARS_ORDER_HADD_UP) {
        bool descending_order = first_pick == PickSplit::LANDMARKS_VARS_ORDER_HADD_DOWN ||
            tiebreak_pick == PickSplit::LANDMARKS_VARS_ORDER_HADD_DOWN ||
            sequence_pick == PickSequenceFlaw::LANDMARKS_VARS_ORDER_HADD_DOWN ||
            sequence_tiebreak_pick == PickSequenceFlaw::LANDMARKS_VARS_ORDER_HADD_DOWN;
        utils::HashSet<int> remaining_vars{};
        for (int i = 0; i < task->get_num_variables(); i++) {
            remaining_vars.insert(i);
        }
        shared_ptr<landmarks::LandmarkGraph> landmark_graph =
            get_landmark_graph(task);
        vector<FactPair> landmark_facts = get_fact_landmarks(*landmark_graph);
        // The rng is not used but needed for the function call.
        utils::RandomNumberGenerator rng{};
        utils::LogProxy log{make_shared<utils::Log>(utils::Verbosity::NORMAL)};
        if (descending_order) {
            filter_and_order_facts(task, FactOrder::HADD_DOWN,
                                   landmark_facts,
                                   rng,
                                   log);
        } else {
            filter_and_order_facts(task, FactOrder::HADD_UP,
                                   landmark_facts,
                                   rng,
                                   log);
        }
        vector<int> sortered_vars = vector<int>{};
        for (FactPair landmark : landmark_facts) {
            if (remaining_vars.contains(landmark.var)) {
                remaining_vars.erase(landmark.var);
                sortered_vars.push_back(landmark.var);
            }
        }
        vector<int> cg_ordered_remaining_vars{};
        if (!remaining_vars.empty()) {
            cg_ordered_remaining_vars.insert(cg_ordered_remaining_vars.end(), remaining_vars.begin(), remaining_vars.end());
            if (descending_order) {
                sort(cg_ordered_remaining_vars.begin(), cg_ordered_remaining_vars.end(), std::greater<int>());
            } else {
                sort(cg_ordered_remaining_vars.begin(), cg_ordered_remaining_vars.end());
            }
            for (int var : cg_ordered_remaining_vars) {
                sortered_vars.push_back(var);
            }
        }
        vars_order = invert_vector(sortered_vars);
    }
    if (first_pick == PickSplit::MAX_POTENTIAL_VARS_ORDER || tiebreak_pick == PickSplit::MAX_POTENTIAL_VARS_ORDER ||
        sequence_pick == PickSequenceFlaw::MAX_POTENTIAL_VARS_ORDER ||
        sequence_tiebreak_pick == PickSequenceFlaw::MAX_POTENTIAL_VARS_ORDER ||
        first_pick == PickSplit::MIN_POTENTIAL_VARS_ORDER || tiebreak_pick == PickSplit::MIN_POTENTIAL_VARS_ORDER ||
        sequence_pick == PickSequenceFlaw::MIN_POTENTIAL_VARS_ORDER ||
        sequence_tiebreak_pick == PickSequenceFlaw::MIN_POTENTIAL_VARS_ORDER) {
        bool descending_order = first_pick == PickSplit::MAX_POTENTIAL_VARS_ORDER ||
            tiebreak_pick == PickSplit::MAX_POTENTIAL_VARS_ORDER ||
            sequence_pick == PickSequenceFlaw::MAX_POTENTIAL_VARS_ORDER ||
            sequence_tiebreak_pick == PickSequenceFlaw::MAX_POTENTIAL_VARS_ORDER;

        potentials::PotentialOptimizer optimizer(
            task, lp_solver, 1e8);
        optimizer.optimize_for_all_states();
        std::vector<std::vector<double>> fact_potentials = optimizer.get_fact_potentials();
        // Use the maximum fact potential for each variable.
        vars_order = vector<int>{};
        vector<pair<int, double>> vars_potential = vector<pair<int, double>>{};
        for (int i = 0; i < task->get_num_variables(); i++) {
            vars_potential.push_back(make_pair(i, *max_element(fact_potentials[i].begin(), fact_potentials[i].end())));
        }

        if (descending_order) {
            sort(vars_potential.begin(), vars_potential.end(), [](pair<int, double> a, pair<int, double> b) {
                     return a.second > b.second;
                 });
        } else {
            sort(vars_potential.begin(), vars_potential.end(), [](pair<int, double> a, pair<int, double> b) {
                     return a.second < b.second;
                 });
        }

        for (auto pair : vars_potential) {
            vars_order.push_back(pair.first);
        }
    }
}

// Define here to avoid include in header.
SplitSelector::~SplitSelector() {
}

int SplitSelector::get_num_unwanted_values(
    const AbstractState &state, const Split &split) const {
    int num_unwanted_values = state.count(split.var_id) - split.values.size();
    assert(num_unwanted_values >= 1);
    return num_unwanted_values;
}

double SplitSelector::get_refinedness(const AbstractState &state, int var_id) const {
    double all_values = task_proxy.get_variables()[var_id].get_domain_size();
    assert(all_values >= 2);
    double remaining_values = state.count(var_id);
    assert(2 <= remaining_values && remaining_values <= all_values);
    double refinedness = -(remaining_values / all_values);
    assert(-1.0 <= refinedness && refinedness < 0.0);
    return refinedness;
}

int SplitSelector::get_hadd_value(int var_id, int value) const {
    assert(additive_heuristic);
    int hadd = additive_heuristic->get_cost_for_cegar(var_id, value);
    assert(hadd != -1);
    return hadd;
}

int SplitSelector::get_min_hadd_value(int var_id, const vector<int> &values) const {
    int min_hadd = numeric_limits<int>::max();
    for (int value : values) {
        const int hadd = get_hadd_value(var_id, value);
        if (hadd < min_hadd) {
            min_hadd = hadd;
        }
    }
    return min_hadd;
}

int SplitSelector::get_max_hadd_value(int var_id, const vector<int> &values) const {
    int max_hadd = -1;
    for (int value : values) {
        const int hadd = get_hadd_value(var_id, value);
        if (hadd > max_hadd) {
            max_hadd = hadd;
        }
    }
    return max_hadd;
}

double SplitSelector::rate_split(
    const AbstractState &state, const Split &split, PickSplit pick, Cost optimal_abstract_plan_cost) const {
    int var_id = split.var_id;
    double rating;
    switch (pick) {
    case PickSplit::MIN_UNWANTED:
        rating = -get_num_unwanted_values(state, split);
        break;
    case PickSplit::MAX_UNWANTED:
        rating = get_num_unwanted_values(state, split);
        break;
    case PickSplit::MIN_REFINED:
        rating = -get_refinedness(state, var_id);
        break;
    case PickSplit::MAX_REFINED:
        rating = get_refinedness(state, var_id);
        break;
    case PickSplit::MIN_HADD:
        rating = -get_min_hadd_value(var_id, split.values);
        break;
    case PickSplit::MAX_HADD:
        rating = get_max_hadd_value(var_id, split.values);
        break;
    case PickSplit::MIN_CG:
        rating = -var_id;
        break;
    case PickSplit::MAX_CG:
        rating = var_id;
        break;
    case PickSplit::HIGHEST_COST_OPERATOR:
        // Prefer splitting goal/initial states to 0-cost operators.
        if (split.op_cost == -1) {
            rating = 0.5;
        } else {
            rating = split.op_cost;
        }
        break;
    case PickSplit::LOWEST_COST_OPERATOR:
        // Prefer operators over initial/goal states.
        if (split.op_cost == 1) {
            rating = -INF;
        } else {
            rating = -split.op_cost;
        }
        break;
    case PickSplit::RANDOM_VARS_ORDER:
    case PickSplit::LANDMARKS_VARS_ORDER_HADD_DOWN:
    case PickSplit::LANDMARKS_VARS_ORDER_HADD_UP:
    case PickSplit::MAX_POTENTIAL_VARS_ORDER:
    case PickSplit::MIN_POTENTIAL_VARS_ORDER:
        rating = -vars_order[var_id];
        break;
    case PickSplit::GOAL_DISTANCE_INCREASED:
    {
        int state_id = state.get_id();
        Cost current_dist = shortest_paths.get_64bit_goal_distance(state_id);
        SimulatedRefinement ref =
            abstraction.simulate_refinement(simulated_transition_system, state, var_id, split.values);
        shortest_paths.update_incrementally(ref.transition_system->get_incoming_transitions(),
                                            ref.transition_system->get_outgoing_transitions(),
                                            state_id,
                                            ref.v1_id,
                                            ref.v2_id,
                                            ref.goals,
                                            0,
                                            true);
        rating = max(shortest_paths.get_64bit_goal_distance(ref.v1_id, true),
                     shortest_paths.get_64bit_goal_distance(ref.v2_id, true)) - current_dist;
        break;
    }
    case PickSplit::OPTIMAL_PLAN_COST_INCREASED:
    {
        int state_id = state.get_id();
        SimulatedRefinement ref =
            abstraction.simulate_refinement(simulated_transition_system, state, var_id, split.values);
        shortest_paths.update_incrementally(ref.transition_system->get_incoming_transitions(),
                                            ref.transition_system->get_outgoing_transitions(),
                                            state_id,
                                            ref.v1_id,
                                            ref.v2_id,
                                            ref.goals,
                                            0,
                                            true);
        Solution solution = *shortest_paths.extract_solution(0, ref.goals, true);
        rating = get_optimal_plan_cost(solution, task_proxy) - optimal_abstract_plan_cost;
        break;
    }
    case PickSplit::BALANCE_REFINED_CLOSEST_GOAL:
    {
        int int_init_dist = shortest_paths.get_64bit_goal_distance(0);
        double init_dist = int_init_dist == 0 ? 1.0 : (double)int_init_dist;
        // Refinedness is negative between 0 and -1.
        // The initial state is always 0 and to normalize between 0 and 1
        // its distance is used as the maximum of the optimal abstract plan.
        rating = get_refinedness(state, var_id) -
            ((double)shortest_paths.get_64bit_goal_distance(state.get_id()) / init_dist);
        break;
    }
    default:
        cerr << "Invalid pick strategy for rate_split(): "
             << static_cast<int>(pick) << endl;
        utils::exit_with(utils::ExitCode::SEARCH_INPUT_ERROR);
    }
    return rating;
}

vector<Split> SplitSelector::compute_max_cover_splits(
    vector<vector<Split>> &&splits) const {
    vector<int> domain_sizes = get_domain_sizes(task_proxy);

    if (debug) {
        cout << "Unsorted splits: " << endl;
        for (auto &var_splits : splits) {
            if (!var_splits.empty()) {
                utils::g_log << " " << var_splits << endl;
            }
        }
    }

    for (auto &var_splits : splits) {
        if (var_splits.size() <= 1) {
            continue;
        }
        // Sort splits by the number of covered flaws.
        sort(var_splits.begin(), var_splits.end(),
             [](const Split &split1, const Split &split2) {
                 return split1.count > split2.count;
             });
        // Try to merge each split into first split.
        Split &best_split_for_var = var_splits[0];
        for (size_t i = 1; i < var_splits.size(); ++i) {
            if (debug) {
                cout << "Combine " << best_split_for_var << " with " << var_splits[i];
            }
            bool combined = best_split_for_var.combine_with(move(var_splits[i]));
            if (debug) {
                cout << " --> " << combined << endl;
            }
            if (combined) {
                var_splits[0].count += var_splits[i].count;
            }
        }
        var_splits.erase(var_splits.begin() + 1, var_splits.end());
    }

    if (debug) {
        cout << "Sorted and combined splits: " << endl;
        for (auto &var_splits : splits) {
            if (!var_splits.empty()) {
                utils::g_log << " " << var_splits << endl;
            }
        }
    }

    vector<Split> best_splits;
    int max_count = -1;
    for (auto &var_splits : splits) {
        if (!var_splits.empty()) {
            Split &best_split_for_var = var_splits[0];
            if (best_split_for_var.count > max_count) {
                best_splits.clear();
                best_splits.push_back(move(best_split_for_var));
                max_count = best_split_for_var.count;
            } else if (best_split_for_var.count == max_count) {
                best_splits.push_back(move(best_split_for_var));
            }
        }
    }
    return best_splits;
}

vector<Split> SplitSelector::reduce_to_best_splits(
    const AbstractState &abstract_state,
    vector<vector<Split>> &&splits,
    Cost optimal_abstract_plan_cost) const {
    if (first_pick == PickSplit::MAX_COVER) {
        return compute_max_cover_splits(move(splits));
    }

    vector<Split> best_splits;
    double max_rating = numeric_limits<double>::lowest();
    for (auto &var_splits : splits) {
        if (!var_splits.empty()) {
            for (Split &split : var_splits) {
                double rating = rate_split(abstract_state, split, first_pick, optimal_abstract_plan_cost);
                if (rating > max_rating) {
                    best_splits.clear();
                    best_splits.push_back(move(split));
                    max_rating = rating;
                } else if (rating == max_rating) {
                    best_splits.push_back(move(split));
                }
            }
        }
    }
    assert(!best_splits.empty());
    return best_splits;
}

Split SplitSelector::select_from_best_splits(
    const AbstractState &abstract_state,
    vector<Split> &&splits,
    Cost optimal_abstract_plan_cost,
    utils::RandomNumberGenerator &rng) const {
    assert(!splits.empty());
    if (splits.size() == 1) {
        return move(splits[0]);
    } else if (tiebreak_pick == PickSplit::RANDOM) {
        return move(*rng.choose(splits));
    }
    double max_rating = numeric_limits<double>::lowest();
    Split *selected_split = nullptr;
    for (Split &split : splits) {
        double rating = rate_split(abstract_state, split, tiebreak_pick, optimal_abstract_plan_cost);
        if (rating > max_rating) {
            selected_split = &split;
            max_rating = rating;
        }
    }
    assert(selected_split);
    return move(*selected_split);
}

Split SplitSelector::pick_split(
    const AbstractState &abstract_state,
    vector<vector<Split>> &&splits,
    Cost optimal_abstract_plan_cost,
    utils::RandomNumberGenerator &rng) const {
    if (first_pick == PickSplit::RANDOM) {
        vector<int> vars_with_splits;
        for (size_t var = 0; var < splits.size(); ++var) {
            auto &var_splits = splits[var];
            if (!var_splits.empty()) {
                vars_with_splits.push_back(var);
            }
        }
        int random_var = *rng.choose(vars_with_splits);
        return move(*rng.choose(splits[random_var]));
    }

    vector<Split> best_splits = reduce_to_best_splits(abstract_state, move(splits), optimal_abstract_plan_cost);
    assert(!best_splits.empty());
    if (debug) {
        utils::g_log << "Best splits: " << best_splits << endl;
    }
    Split selected_split = select_from_best_splits(abstract_state, move(best_splits), optimal_abstract_plan_cost, rng);
    if (debug) {
        utils::g_log << "Selected split: " << selected_split << endl;
    }
    return selected_split;
}

static plugins::TypedEnumPlugin<PickSplit> _enum_plugin({
        {"random",
         "select a random variable (among all eligible variables)"},
        {"min_unwanted",
         "select an eligible variable which has the least unwanted values "
         "(number of values of v that land in the abstract state whose "
         "h-value will probably be raised) in the flaw state"},
        {"max_unwanted",
         "select an eligible variable which has the most unwanted values "
         "(number of values of v that land in the abstract state whose "
         "h-value will probably be raised) in the flaw state"},
        {"min_refined",
         "select an eligible variable which is the least refined "
         "(-1 * (remaining_values(v) / original_domain_size(v))) "
         "in the flaw state"},
        {"max_refined",
         "select an eligible variable which is the most refined "
         "(-1 * (remaining_values(v) / original_domain_size(v))) "
         "in the flaw state"},
        {"min_hadd",
         "select an eligible variable with minimal h^add(s_0) value "
         "over all facts that need to be removed from the flaw state"},
        {"max_hadd",
         "select an eligible variable with maximal h^add(s_0) value "
         "over all facts that need to be removed from the flaw state"},
        {"min_cg",
         "order by increasing position in partial ordering of causal graph"},
        {"max_cg",
         "order by decreasing position in partial ordering of causal graph"},
        {"max_cover",
         "compute split that covers the maximum number of flaws for several concrete states."},
        {"highest_cost_operator", "the operator with the highest cost"},
        {"lowest_cost_operator", "the operator with the lowest cost"},
        {"random_vars_order", "random order of variables"},
        {"landmarks_vars_order_hadd_down", "landmarks order of variables sorted by h^{add} in descending order"},
        {"landmarks_vars_order_hadd_up", "landmarks order of variables sorted by h^{add} in ascending order"},
        {"max_potential_vars_order", "max potential order of variables (the max of all facts is used for each variable)"},
        {"min_potential_vars_order", "min potential order of variables (the max of all facts is used for each variable)"},
        {"goal_distance_increased",
         "amount in which the distance to goal is increased after the refinement."},
        {"optimal_plan_cost_increased",
         "amount in which the cost of the optimal plan is increased after the refinement."},
        {"balance_refined_closest_goal",
         "max_refined and distance of the state before refinement to goal with the same weight."}
    });
static plugins::TypedEnumPlugin<PickSequenceFlaw> _enum_plugin_sequence({
        {"random",
         "select a random variable (among all eligible variables)"},
        {"min_unwanted",
         "select an eligible variable which has the least unwanted values "
         "(number of values of v that land in the abstract state whose "
         "h-value will probably be raised) in the flaw state"},
        {"max_unwanted",
         "select an eligible variable which has the most unwanted values "
         "(number of values of v that land in the abstract state whose "
         "h-value will probably be raised) in the flaw state"},
        {"min_refined",
         "select an eligible variable which is the least refined "
         "(-1 * (remaining_values(v) / original_domain_size(v))) "
         "in the flaw state"},
        {"max_refined",
         "select an eligible variable which is the most refined "
         "(-1 * (remaining_values(v) / original_domain_size(v))) "
         "in the flaw state"},
        {"min_hadd",
         "select an eligible variable with minimal h^add(s_0) value "
         "over all facts that need to be removed from the flaw state"},
        {"max_hadd",
         "select an eligible variable with maximal h^add(s_0) value "
         "over all facts that need to be removed from the flaw state"},
        {"min_cg",
         "order by increasing position in partial ordering of causal graph"},
        {"max_cg",
         "order by decreasing position in partial ordering of causal graph"},
        {"max_cover",
         "compute split that covers the maximum number of flaws for several concrete states."},
        {"highest_cost_operator", "the operator with the highest cost"},
        {"lowest_cost_operator", "the operator with the lowest cost"},
        {"random_vars_order", "random order of variables"},
        {"landmarks_vars_order_hadd_down", "landmarks order of variables sorted by h^{add} in descending order"},
        {"landmarks_vars_order_hadd_up", "landmarks order of variables sorted by h^{add} in ascending order"},
        {"max_potential_vars_order", "max potential order of variables (the max of all facts is used for each variable)"},
        {"min_potential_vars_order", "min potential order of variables (the max of all facts is used for each variable)"},
        {"goal_distance_increased",
         "amount in which the distance to goal is increased after the refinement."},
        {"optimal_plan_cost_increased",
         "amount in which the cost of the optimal plan is increased after the refinement."},
        {"balance_refined_closest_goal",
         "max_refined and distance of the state before refinement to goal with the same weight."},
        {"first_flaw", "the first flaw found"},
        {"last_flaw", "the last flaw found"},
        {"closest_to_goal_flaw", "the flaw closest to the goal state"}
    });
}
