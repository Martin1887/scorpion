#include "cartesian_abstraction_generator.h"

#include "explicit_abstraction.h"
#include "types.h"

#include "../option_parser.h"
#include "../plugin.h"

#include "../cegar/abstraction.h"
#include "../cegar/abstract_search.h"
#include "../cegar/abstract_state.h"
#include "../cegar/cegar.h"
#include "../cegar/refinement_hierarchy.h"
#include "../cegar/split_selector.h"
#include "../cegar/subtask_generators.h"
#include "../cegar/transition_system.h"
#include "../task_utils/task_properties.h"
#include "../utils/logging.h"
#include "../utils/rng_options.h"

#include <memory>

using namespace std;

namespace cost_saturation {
CartesianAbstractionGenerator::CartesianAbstractionGenerator(
    const options::Options &opts)
    : subtask_generators(
          opts.get_list<shared_ptr<cegar::SubtaskGenerator>>("subtasks")),
      max_states(opts.get<int>("max_states")),
      max_transitions(opts.get<int>("max_transitions")),
      rng(utils::parse_rng_from_options(opts)),
      debug(opts.get<bool>("debug")),
      num_states(0),
      num_transitions(0) {
}

static vector<int> get_looping_operators(const cegar::TransitionSystem &ts) {
    int num_operators = ts.get_num_operators();

    vector<bool> operator_induces_self_loop(num_operators, false);
    for (const auto &loops : ts.get_loops()) {
        for (int op_id : loops) {
            operator_induces_self_loop[op_id] = true;
        }
    }

    vector<int> looping_operators;
    for (int op_id = 0; op_id < num_operators; ++op_id) {
        if (operator_induces_self_loop[op_id]) {
            looping_operators.push_back(op_id);
        }
    }
    looping_operators.shrink_to_fit();
    return looping_operators;
}


static unique_ptr<Abstraction> convert_abstraction(
    cegar::Abstraction &cartesian_abstraction, const vector<int> &operator_costs) {
    // Compute g and h values.
    const cegar::TransitionSystem &ts =
        cartesian_abstraction.get_transition_system();
    cegar::AbstractSearch search(operator_costs);
    int initial_state_id = cartesian_abstraction.get_initial_state()->get_id();
    vector<int> g_values = search.compute_distances(
        ts.get_outgoing_transitions(), {initial_state_id});
    vector<int> h_values = search.compute_distances(
        ts.get_incoming_transitions(), cartesian_abstraction.get_goals());

    // Retrieve non-looping transitions.
    vector<vector<Successor>> backward_graph(cartesian_abstraction.get_num_states());
    for (int state_id = 0; state_id < cartesian_abstraction.get_num_states(); ++state_id) {
        // Ignore transitions from dead-end or unreachable states.
        if (h_values[state_id] == INF || g_values[state_id] == INF) {
            continue;
        }
        for (const cegar::Transition &transition : ts.get_outgoing_transitions()[state_id]) {
            // Ignore transitions from dead-end states (we know target is reachable).
            if (h_values[transition.target_id] == INF) {
                continue;
            }
            backward_graph[transition.target_id].emplace_back(transition.op_id, state_id);
        }
    }
    for (vector<Successor> &succesors : backward_graph) {
        succesors.shrink_to_fit();
    }

    vector<int> looping_operators = get_looping_operators(ts);
    vector<int> goal_states(
        cartesian_abstraction.get_goals().begin(),
        cartesian_abstraction.get_goals().end());

    // Convert to shared_ptr since std::function requires copy-constructible arguments.
    shared_ptr<cegar::RefinementHierarchy> refinement_hierarchy =
        cartesian_abstraction.extract_refinement_hierarchy();
    AbstractionFunction state_map =
        [refinement_hierarchy](const State &state) {
            assert(refinement_hierarchy);
            return refinement_hierarchy->get_abstract_state_id(state);
        };

    return utils::make_unique_ptr<ExplicitAbstraction>(
        state_map,
        move(backward_graph),
        move(looping_operators),
        move(goal_states));
}

void CartesianAbstractionGenerator::build_abstractions_for_subtasks(
    const vector<shared_ptr<AbstractTask>> &subtasks,
    function<bool()> total_size_limit_reached,
    Abstractions &abstractions) {
    int remaining_subtasks = subtasks.size();
    for (const shared_ptr<AbstractTask> &subtask : subtasks) {
        /* To make the abstraction refinement process deterministic, we don't
           set a time limit. */
        const double max_time = numeric_limits<double>::infinity();

        cegar::CEGAR cegar(
            subtask,
            max(1, (max_states - num_states) / remaining_subtasks),
            max(1, (max_transitions - num_transitions) / remaining_subtasks),
            max_time,
            cegar::PickSplit::MAX_REFINED,
            *rng,
            debug);

        unique_ptr<cegar::Abstraction> cartesian_abstraction = cegar.extract_abstraction();

        num_states += cartesian_abstraction->get_num_states();
        num_transitions += cartesian_abstraction->get_transition_system().get_num_non_loops();
        int init_h = cartesian_abstraction->get_initial_state()->get_goal_distance_estimate();

        vector<int> operator_costs = task_properties::get_operator_costs(TaskProxy(*subtask));
        abstractions.push_back(convert_abstraction(*cartesian_abstraction, operator_costs));

        if (total_size_limit_reached() || init_h == INF) {
            break;
        }

        --remaining_subtasks;
    }
}

Abstractions CartesianAbstractionGenerator::generate_abstractions(
    const shared_ptr<AbstractTask> &task) {
    utils::Timer timer;
    utils::Log log;
    log << "Build Cartesian abstractions" << endl;

    // TODO: The CEGAR code expects some extra memory padding to be reserved.
    // Using memory padding leads to nondeterministic results. Therefore, we
    // want to get rid of the memory padding here and in the CEGAR code.
    utils::reserve_extra_memory_padding(0);

    function<bool()> total_size_limit_reached =
        [&] () {
            return num_transitions >= max_transitions;
        };

    Abstractions abstractions;
    for (const auto &subtask_generator : subtask_generators) {
        cegar::SharedTasks subtasks = subtask_generator->get_subtasks(task);
        build_abstractions_for_subtasks(
            subtasks, total_size_limit_reached, abstractions);
        if (total_size_limit_reached()) {
            break;
        }
    }

    if (utils::extra_memory_padding_is_reserved()) {
        utils::release_extra_memory_padding();
    }

    log << "Cartesian abstractions built: " << abstractions.size() << endl;
    log << "Time for building Cartesian abstractions: " << timer << endl;
    return abstractions;
}

static shared_ptr<AbstractionGenerator> _parse(OptionParser &parser) {
    parser.document_synopsis(
        "Cartesian abstraction generator",
        "");

    parser.add_list_option<shared_ptr<cegar::SubtaskGenerator>>(
        "subtasks",
        "subtask generators",
        "[landmarks(order=random, random_seed=0), goals(order=random, random_seed=0)]");
    parser.add_option<int>(
        "max_states",
        "maximum sum of abstract states over all abstractions",
        "infinity",
        Bounds("0", "infinity"));
    parser.add_option<int>(
        "max_transitions",
        "maximum sum of state-changing transitions (excluding self-loops) over "
        " all abstractions",
        "1000000",
        Bounds("0", "infinity"));
    parser.add_option<bool>(
        "debug",
        "print debugging info",
        "false");
    utils::add_rng_options(parser);

    Options opts = parser.parse();
    if (parser.dry_run())
        return nullptr;

    return make_shared<CartesianAbstractionGenerator>(opts);
}

static PluginShared<AbstractionGenerator> _plugin("cartesian", _parse);
}
