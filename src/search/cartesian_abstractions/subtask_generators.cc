#include "subtask_generators.h"

#include "split_selector.h"
#include "utils.h"
#include "utils_landmarks.h"

#include "../heuristics/additive_heuristic.h"
#include "../landmarks/landmark_graph.h"
#include "../plugins/plugin.h"
#include "../task_utils/task_properties.h"
#include "../tasks/domain_abstracted_task_factory.h"
#include "../tasks/modified_goals_task.h"
#include "../utils/logging.h"
#include "../utils/rng.h"
#include "../utils/rng_options.h"

#include <algorithm>
#include <cassert>
#include <iostream>
#include <string>
#include <unordered_set>
#include <vector>

using namespace std;

namespace cartesian_abstractions {
class SortFactsByIncreasingHaddValues {
    // Can't store as unique_ptr since the class needs copy-constructor.
    shared_ptr<additive_heuristic::AdditiveHeuristic> hadd;

    int get_cost(const FactPair &fact) const {
        return hadd->get_cost_for_cegar(fact.var, fact.value);
    }

public:
    explicit SortFactsByIncreasingHaddValues(
        const shared_ptr<AbstractTask> &task)
        : hadd(create_additive_heuristic(task)) {
        TaskProxy task_proxy(*task);
        hadd->compute_heuristic_for_cegar(task_proxy.get_initial_state());
    }

    bool operator()(const FactPair &a, const FactPair &b) {
        return get_cost(a) < get_cost(b);
    }
};


static void remove_initial_state_facts(
    const TaskProxy &task_proxy, Facts &facts) {
    State initial_state = task_proxy.get_initial_state();
    facts.erase(remove_if(facts.begin(), facts.end(), [&](FactPair fact) {
                              return initial_state[fact.var].get_value() == fact.value;
                          }), facts.end());
}

static void order_facts(
    const shared_ptr<AbstractTask> &task,
    FactOrder fact_order,
    vector<FactPair> &facts,
    utils::RandomNumberGenerator &rng,
    utils::LogProxy &log) {
    if (log.is_at_least_verbose()) {
        log << "Sort " << facts.size() << " facts" << endl;
    }
    switch (fact_order) {
    case FactOrder::ORIGINAL:
        // Nothing to do.
        break;
    case FactOrder::RANDOM:
        rng.shuffle(facts);
        break;
    case FactOrder::HADD_UP:
    case FactOrder::HADD_DOWN:
        sort(facts.begin(), facts.end(), SortFactsByIncreasingHaddValues(task));
        if (fact_order == FactOrder::HADD_DOWN)
            reverse(facts.begin(), facts.end());
        break;
    default:
        cerr << "Invalid task order: " << static_cast<int>(fact_order) << endl;
        utils::exit_with(utils::ExitCode::SEARCH_INPUT_ERROR);
    }
}

Facts filter_and_order_facts(
    const shared_ptr<AbstractTask> &task,
    FactOrder fact_order,
    Facts &facts,
    utils::RandomNumberGenerator &rng,
    utils::LogProxy &log) {
    TaskProxy task_proxy(*task);
    remove_initial_state_facts(task_proxy, facts);
    order_facts(task, fact_order, facts, rng, log);
    return facts;
}


TaskDuplicator::TaskDuplicator(const plugins::Options &opts)
    : SameParamsSubtaskGenerator(opts),
      num_copies(opts.get<int>("copies")) {
}

SharedTasks TaskDuplicator::get_subtasks(
    const shared_ptr<AbstractTask> &task, utils::LogProxy &) const {
    Subtask subtask {
        .subproblem_id = 0,
        .subtask = task,
        .pick_flawed_abstract_state = pick_flawed_abstract_state,
        .pick_split = pick_split,
        .tiebreak_split = tiebreak_split,
        .sequence_split = sequence_split,
        .sequence_tiebreak_split = sequence_tiebreak_split,
        .intersect_flaw_search_abstract_states = intersect_flaw_search_abstract_states
    };
    SharedTasks subtasks;
    subtasks.reserve(num_copies);
    for (int i = 0; i < num_copies; ++i) {
        subtasks.push_back(subtask);
    }
    return subtasks;
}

GoalDecomposition::GoalDecomposition(const plugins::Options &opts)
    : SameParamsSubtaskGenerator(opts),
      fact_order(opts.get<FactOrder>("order")),
      rng(utils::parse_rng_from_options(opts)) {
}

SharedTasks GoalDecomposition::get_subtasks(
    const shared_ptr<AbstractTask> &task, utils::LogProxy &log) const {
    SharedTasks subtasks;
    TaskProxy task_proxy(*task);
    Facts goal_facts = task_properties::get_fact_pairs(task_proxy.get_goals());
    filter_and_order_facts(task, fact_order, goal_facts, *rng, log);
    int i = 0;
    for (const FactPair &goal : goal_facts) {
        shared_ptr<AbstractTask> subproblem =
            make_shared<extra_tasks::ModifiedGoalsTask>(task, Facts {goal});
        Subtask subtask {
            .subproblem_id = i,
            .subtask = subproblem,
            .pick_flawed_abstract_state = pick_flawed_abstract_state,
            .pick_split = pick_split,
            .tiebreak_split = tiebreak_split,
            .sequence_split = sequence_split,
            .sequence_tiebreak_split = sequence_tiebreak_split,
            .intersect_flaw_search_abstract_states = intersect_flaw_search_abstract_states
        };
        subtasks.push_back(subtask);
        i++;
    }
    return subtasks;
}


LandmarkDecomposition::LandmarkDecomposition(const plugins::Options &opts)
    : SameParamsSubtaskGenerator(opts),
      fact_order(opts.get<FactOrder>("order")),
      combine_facts(opts.get<bool>("combine_facts")),
      rng(utils::parse_rng_from_options(opts)) {
}

shared_ptr<AbstractTask> LandmarkDecomposition::build_domain_abstracted_task(
    const shared_ptr<AbstractTask> &parent,
    const landmarks::LandmarkGraph &landmark_graph,
    const FactPair &fact) const {
    assert(combine_facts);
    extra_tasks::VarToGroups value_groups;
    for (const auto &pair : get_prev_landmarks(landmark_graph, fact)) {
        int var = pair.first;
        const vector<int> &group = pair.second;
        if (group.size() >= 2)
            value_groups[var].push_back(group);
    }
    return extra_tasks::build_domain_abstracted_task(parent, value_groups);
}

SharedTasks LandmarkDecomposition::get_subtasks(
    const shared_ptr<AbstractTask> &task, utils::LogProxy &log) const {
    SharedTasks subtasks;
    shared_ptr<landmarks::LandmarkGraph> landmark_graph =
        get_landmark_graph(task);
    Facts landmark_facts = get_fact_landmarks(*landmark_graph);
    filter_and_order_facts(task, fact_order, landmark_facts, *rng, log);
    int i = 0;
    for (const FactPair &landmark : landmark_facts) {
        shared_ptr<AbstractTask> subproblem =
            make_shared<extra_tasks::ModifiedGoalsTask>(task, Facts {landmark});
        if (combine_facts) {
            subproblem = build_domain_abstracted_task(
                subproblem, *landmark_graph, landmark);
        }
        Subtask subtask {
            .subproblem_id = i,
            .subtask = subproblem,
            .pick_flawed_abstract_state = pick_flawed_abstract_state,
            .pick_split = pick_split,
            .tiebreak_split = tiebreak_split,
            .sequence_split = sequence_split,
            .sequence_tiebreak_split = sequence_tiebreak_split,
            .intersect_flaw_search_abstract_states = intersect_flaw_search_abstract_states
        };
        subtasks.push_back(subtask);
        i++;
    }
    return subtasks;
}

VarsOrdersSubtaskGenerator::VarsOrdersSubtaskGenerator(const plugins::Options &opts)
    : DiversifiedSubtaskGenerator(opts) {
}

SharedTasks VarsOrdersSubtaskGenerator::get_subtasks(
    const shared_ptr<AbstractTask> &task, utils::LogProxy &log) const {
    SharedTasks subtasks;
    vector<PickSplit> vars_orders = {
        PickSplit::MAX_CG,
        PickSplit::MIN_CG,
        PickSplit::LANDMARKS_VARS_ORDER_HADD_DOWN,
        PickSplit::LANDMARKS_VARS_ORDER_HADD_UP,
        PickSplit::MAX_POTENTIAL_VARS_ORDER,
        PickSplit::MIN_POTENTIAL_VARS_ORDER,
    };
    log << "Vars orders diversification with orders " << vars_orders << endl;
    for (PickSplit order : vars_orders) {
        subtasks.push_back(
            Subtask {
                .subproblem_id = 0,
                .subtask = task,
                .pick_flawed_abstract_state = pick_flawed_abstract_state,
                .pick_split = order,
                .tiebreak_split = tiebreak_split,
                .sequence_split = PickSequenceFlaw::BEST_SPLIT,
                .sequence_tiebreak_split = PickSequenceFlaw::BEST_SPLIT,
                .intersect_flaw_search_abstract_states = intersect_flaw_search_abstract_states
            }
            );
    }

    return subtasks;
}

BestStrategiesSubtaskGenerator::BestStrategiesSubtaskGenerator(const plugins::Options &opts)
    : DiversifiedSubtaskGenerator(opts) {
}

SharedTasks BestStrategiesSubtaskGenerator::get_subtasks(
    const shared_ptr<AbstractTask> &task, utils::LogProxy &log) const {
    SharedTasks subtasks;
    subtasks.push_back(
        Subtask {
            .subproblem_id = 0,
            .subtask = task,
            .pick_flawed_abstract_state = pick_flawed_abstract_state,
            .pick_split = PickSplit::MAX_COVER,
            .tiebreak_split = tiebreak_split,
            .sequence_split = PickSequenceFlaw::CLOSEST_TO_GOAL_FLAW,
            .sequence_tiebreak_split = PickSequenceFlaw::BEST_SPLIT,
            .intersect_flaw_search_abstract_states = intersect_flaw_search_abstract_states
        });
    vector<PickSplit> best_strategies = {
        PickSplit::MAX_REFINED,
        PickSplit::MAX_CG,
        PickSplit::LANDMARKS_HADD_DOWN,
        PickSplit::LANDMARKS_HADD_UP,
        PickSplit::MAX_POTENTIAL,
        PickSplit::MIN_POTENTIAL,
        PickSplit::GOAL_DISTANCE_INCREASED,
    };
    log << "Best strategies diversification with strategies closest_to_goal, "
        << best_strategies << endl;
    for (PickSplit strategy : best_strategies) {
        subtasks.push_back(
            Subtask {
                .subproblem_id = 0,
                .subtask = task,
                .pick_flawed_abstract_state = pick_flawed_abstract_state,
                .pick_split = strategy,
                .tiebreak_split = tiebreak_split,
                .sequence_split = PickSequenceFlaw::BEST_SPLIT,
                .sequence_tiebreak_split = PickSequenceFlaw::BEST_SPLIT,
                .intersect_flaw_search_abstract_states = intersect_flaw_search_abstract_states
            });
    }

    return subtasks;
}

static void add_fact_order_option(plugins::Feature &feature) {
    feature.add_option<FactOrder>(
        "order",
        "ordering of goal or landmark facts",
        "hadd_down");
    utils::add_rng_options(feature);
}

static void add_diversified_base_options(plugins::Feature &feature) {
    feature.add_option<cartesian_abstractions::PickFlawedAbstractState>(
        "pick_flawed_abstract_state",
        "flaw-selection strategy",
        "batch_min_h");
    feature.add_option<PickSplit>(
        "tiebreak_split",
        "split-selection strategy for breaking ties",
        "max_refined");
    feature.add_option<bool>(
        "intersect_flaw_search_abstract_states",
        "intersect flaw search states with the mapped one to find more flaws",
        "false");
}

static void add_all_base_options(plugins::Feature &feature) {
    add_diversified_base_options(feature);
    feature.add_option<PickSplit>(
        "pick_split",
        "split-selection strategy",
        "max_cover");
    feature.add_option<PickSequenceFlaw>(
        "sequence_split",
        "split-selection strategy for choosing among flaws in different states",
        "closest_to_goal_flaw");
    feature.add_option<PickSequenceFlaw>(
        "sequence_tiebreak_split",
        "split-selection strategy for breaking ties when choosing among flaws in different states",
        "best_split");
}

class TaskDuplicatorFeature : public plugins::TypedFeature<SubtaskGenerator, TaskDuplicator> {
public:
    TaskDuplicatorFeature() : TypedFeature("original") {
        add_all_base_options(*this);
        add_option<int>(
            "copies",
            "number of task copies",
            "1",
            plugins::Bounds("1", "infinity"));
    }
};

static plugins::FeaturePlugin<TaskDuplicatorFeature> _plugin_original;

class GoalDecompositionFeature : public plugins::TypedFeature<SubtaskGenerator, GoalDecomposition> {
public:
    GoalDecompositionFeature() : TypedFeature("goals") {
        add_all_base_options(*this);
        add_fact_order_option(*this);
    }
};

static plugins::FeaturePlugin<GoalDecompositionFeature> _plugin_goals;


class LandmarkDecompositionFeature : public plugins::TypedFeature<SubtaskGenerator, LandmarkDecomposition> {
public:
    LandmarkDecompositionFeature() : TypedFeature("landmarks") {
        add_all_base_options(*this);
        add_fact_order_option(*this);
        add_option<bool>(
            "combine_facts",
            "combine landmark facts with domain abstraction",
            "true");
    }
};

static plugins::FeaturePlugin<LandmarkDecompositionFeature> _plugin_landmarks;


class VarsOrdersSubtaskGeneratorFeature : public plugins::TypedFeature<SubtaskGenerator, VarsOrdersSubtaskGenerator> {
public:
    VarsOrdersSubtaskGeneratorFeature() : TypedFeature("vars_orders") {
        add_diversified_base_options(*this);
    }
};

static plugins::FeaturePlugin<VarsOrdersSubtaskGeneratorFeature> _plugin_vars_orders;


class BestStrategiesSubtaskGeneratorFeature : public plugins::TypedFeature<SubtaskGenerator, BestStrategiesSubtaskGenerator> {
public:
    BestStrategiesSubtaskGeneratorFeature() : TypedFeature("best_strategies") {
        add_diversified_base_options(*this);
    }
};

static plugins::FeaturePlugin<BestStrategiesSubtaskGeneratorFeature> _plugin_best_strategies;


static class SubtaskGeneratorCategoryPlugin : public plugins::TypedCategoryPlugin<SubtaskGenerator> {
public:
    SubtaskGeneratorCategoryPlugin() : TypedCategoryPlugin("SubtaskGenerator") {
        document_synopsis("Subtask generator (used by the CEGAR heuristic).");
    }
}
_category_plugin;

static plugins::TypedEnumPlugin<FactOrder> _enum_plugin({
        {"original", "according to their (internal) variable index"},
        {"random", "according to a random permutation"},
        {"hadd_up", "according to their h^add value, lowest first"},
        {"hadd_down", "according to their h^add value, highest first "}
    });
}
