#include "flaw_search.h"

#include "abstraction.h"
#include "abstract_state.h"
#include "flaw.h"
#include "transition_system.h"
#include "shortest_paths.h"
#include "split_selector.h"

#include "../option_parser.h"
#include "../plugin.h"

#include "../evaluators/g_evaluator.h"
#include "../open_lists/best_first_open_list.h"
#include "../task_utils/successor_generator.h"
#include "../task_utils/task_properties.h"
#include "../utils/logging.h"
#include "../utils/rng.h"

#include <iterator>
#include <optional.hh>

using namespace std;

namespace cegar {
CartesianSet FlawSearch::get_cartesian_set(const ConditionsProxy &conditions) const {
    CartesianSet cartesian_set(domain_sizes);
    for (FactProxy condition : conditions) {
        cartesian_set.set_single_value(condition.get_variable().get_id(),
                                       condition.get_value());
    }
    return cartesian_set;
}

int FlawSearch::get_abstract_state_id(const State &state) const {
    return abstraction.get_abstract_state_id(state);
}

int FlawSearch::get_h_value(int abstract_state_id) const {
    return shortest_paths.get_goal_distance(abstract_state_id);
}

bool FlawSearch::is_f_optimal_transition(int abstract_state_id,
                                         const Transition &tr) const {
    int source_h_value = get_h_value(abstract_state_id);
    int target_h_value = get_h_value(tr.target_id);
    int op_cost = task_proxy.get_operators()[tr.op_id].get_cost();
    return source_h_value - op_cost == target_h_value;
}

const vector<Transition> &FlawSearch::get_transitions(
    int abstract_state_id) const {
    return abstraction.get_transition_system().
           get_outgoing_transitions().at(abstract_state_id);
}

void FlawSearch::add_flaw(int abs_id, const State &state) {
    // Using a reference to flawed_states[abs_id] doesn't work since it creates a temporary.
    assert(find(flawed_states[abs_id].begin(), flawed_states[abs_id].end(), state) ==
           flawed_states[abs_id].end());
    int h = get_h_value(abs_id);
    if (pick_flaw == PickFlaw::MIN_H_SINGLE
        || pick_flaw == PickFlaw::MIN_H_BATCH
        || pick_flaw == PickFlaw::MIN_H_BATCH_MULTI_SPLIT) {
        if (best_flaw_h > h) {
            flawed_states.clear();
        }
        if (best_flaw_h >= h) {
            best_flaw_h = h;
            flawed_states[abs_id].push_back(state);
        }
    } else if (pick_flaw == PickFlaw::MAX_H_SINGLE) {
        if (best_flaw_h < h) {
            flawed_states.clear();
        }
        if (best_flaw_h <= h) {
            best_flaw_h = h;
            flawed_states[abs_id].push_back(state);
        }
    } else {
        assert(pick_flaw == PickFlaw::RANDOM_H_SINGLE);
        flawed_states[abs_id].push_back(state);
    }
    assert(!flawed_states.empty());
}

void FlawSearch::initialize() {
    ++num_searches;
    last_refined_abstract_state_id = -1;
    best_flaw_h = (pick_flaw == PickFlaw::MAX_H_SINGLE) ? -INF : INF;
    open_list->clear();
    state_registry = utils::make_unique_ptr<StateRegistry>(task_proxy);
    search_space = utils::make_unique_ptr<SearchSpace>(*state_registry);
    statistics = utils::make_unique_ptr<SearchStatistics>(utils::Verbosity::SILENT);

    flawed_states.clear();

    State initial_state = state_registry->get_initial_state();
    EvaluationContext eval_context(initial_state, 0, false, statistics.get());

    if (open_list->is_dead_end(eval_context)) {
        utils::g_log << "Initial state is a dead end." << endl;
    } else {
        SearchNode node = search_space->get_node(initial_state);
        node.open_initial();
        open_list->insert(eval_context, initial_state.get_id());
    }
}

SearchStatus FlawSearch::step() {
    if (open_list->empty()) {
        /// Completely explored state space
        return FAILED;
    }
    StateID id = open_list->remove_min();
    State s = state_registry->lookup_state(id);
    SearchNode node = search_space->get_node(s);

    assert(!node.is_closed());
    assert(node.get_real_g() + get_h_value(get_abstract_state_id(s))
           <= get_h_value(abstraction.get_initial_state().get_id()));

    node.close();
    assert(!node.is_dead_end());
    ++num_overall_expanded_concrete_states;
    statistics->inc_expanded();

    if (task_properties::is_goal_state(task_proxy, s)) {
        return SOLVED;
    }

    bool found_flaw = false;
    int abs_id = get_abstract_state_id(s);

    // Check for each tr if the op is applicable or if there is a deviation
    for (const Transition &tr : get_transitions(abs_id)) {
        if (!utils::extra_memory_padding_is_reserved())
            return TIMEOUT;

        // same f-layer
        if (is_f_optimal_transition(abs_id, tr)) {
            OperatorProxy op = task_proxy.get_operators()[tr.op_id];

            // Applicability flaw
            if (!task_properties::is_applicable(op, s)) {
                if (!found_flaw) {
                    add_flaw(abs_id, s);
                    found_flaw = true;
                }
                if (pick_flaw == PickFlaw::MAX_H_SINGLE) {
                    return FAILED;
                }
                continue;
            }
            State succ_state = state_registry->get_successor_state(s, op);
            // Deviation flaw
            if (!abstraction.get_state(tr.target_id).includes(succ_state)) {
                if (!found_flaw) {
                    add_flaw(abs_id, s);
                    found_flaw = true;
                }
                if (pick_flaw == PickFlaw::MAX_H_SINGLE) {
                    return FAILED;
                }
                continue;
            }

            statistics->inc_generated();
            SearchNode succ_node = search_space->get_node(succ_state);

            assert(!succ_node.is_dead_end());

            if (succ_node.is_new()) {
                // We have not seen this state before.
                // Evaluate and create a new node.

                // Careful: succ_node.get_g() is not available here yet,
                // hence the stupid computation of succ_g.
                // TODO: Make this less fragile.
                int succ_g = node.get_g() + op.get_cost();

                EvaluationContext succ_eval_context(
                    succ_state, succ_g, false, statistics.get());
                statistics->inc_evaluated_states();

                succ_node.open(node, op, op.get_cost());

                open_list->insert(succ_eval_context, succ_state.get_id());
            }
        }
    }
    return IN_PROGRESS;
}

static void get_possible_splits(
    const AbstractState &abs_state,
    const State &conc_state,
    const ConditionsProxy &preconditions,
    vector<Split> &splits) {
    for (FactProxy precondition_proxy : preconditions) {
        FactPair fact = precondition_proxy.get_pair();
        assert(abs_state.contains(fact.var, fact.value));
        int value = conc_state[fact.var].get_value();
        if (value != fact.value) {
            vector<int> wanted = {fact.value};
            splits.emplace_back(abs_state.get_id(), fact.var, value, move(wanted));
        }
    }
    assert(!splits.empty());
}

unique_ptr<Split> FlawSearch::create_split(
    const vector<State> &states, int abstract_state_id) {
    const AbstractState &abstract_state = abstraction.get_state(abstract_state_id);

    vector<Split> splits;
    for (const Transition &tr : get_transitions(abstract_state_id)) {
        if (is_f_optimal_transition(abstract_state_id, tr)) {
            OperatorProxy op = task_proxy.get_operators()[tr.op_id];

            for (const State &state : states) {
                // Applicability flaw
                if (!task_properties::is_applicable(op, state)) {
                    get_possible_splits(abstract_state, state, op.get_preconditions(), splits);
                } else {
                    // Deviation flaw
                    assert(tr.target_id != get_abstract_state_id(
                               state_registry->get_successor_state(state, op)));
                    Flaw flaw(abstract_state, state, abstraction.get_state(tr.target_id).regress(op));
                    get_possible_splits(flaw, splits);
                }
            }
        }
    }

    return split_selector.pick_split(abstract_state, move(splits), rng);
}

SearchStatus FlawSearch::search_for_flaws() {
    initialize();
    size_t cur_expanded_states = num_overall_expanded_concrete_states;
    SearchStatus search_status = IN_PROGRESS;
    while (search_status == IN_PROGRESS) {
        search_status = step();
    }

    if (debug) {
        cout << endl;
        utils::g_log << "Expanded "
                     << num_overall_expanded_concrete_states - cur_expanded_states
                     << " states." << endl;
        utils::g_log << "Flawed States: " << endl;
        if (search_status == FAILED) {
            for (auto const &pair : flawed_states) {
                for (const State &s : pair.second) {
                    utils::g_log << "  <" << pair.first << "," << s.get_id()
                                 << ">: " << *create_split({s}, pair.first)
                                 << endl;
                }
            }
        }
    }
    return search_status;
}

unique_ptr<Split> FlawSearch::get_random_single_split() {
    auto search_status = search_for_flaws();

    if (search_status == FAILED) {
        assert(!flawed_states.empty());

        ++num_overall_refined_flaws;

        int rng_abstract_state_id =
            next(flawed_states.begin(), rng(flawed_states.size()))->first;
        auto rng_state =
            *next(flawed_states.at(rng_abstract_state_id).begin(),
                  rng(flawed_states.at(rng_abstract_state_id).size()));

        auto split = create_split({rng_state}, rng_abstract_state_id);
        best_flaw_h = get_h_value(split->abstract_state_id);
        return split;
    }
    assert(search_status == SOLVED);
    return nullptr;
}

unique_ptr<Split> FlawSearch::get_single_split() {
    auto search_status = search_for_flaws();

    // Memory padding
    if (search_status == TIMEOUT)
        return nullptr;

    if (search_status == FAILED) {
        assert(!flawed_states.empty());

        ++num_overall_refined_flaws;
        int abs_state_id = flawed_states.begin()->first;
        const State &state = *flawed_states.begin()->second.begin();
        if (debug) {
            vector<OperatorID> trace;
            search_space->trace_path(state, trace);
            vector<string> operator_names;
            operator_names.reserve(trace.size());
            for (OperatorID op_id : trace) {
                operator_names.push_back(task_proxy.get_operators()[op_id].get_name());
            }
            utils::g_log << "Path (without last operator): " << operator_names << endl;
        }
        return create_split({state}, abs_state_id);
    }
    assert(search_status == SOLVED);
    return nullptr;
}

unique_ptr<Split>
FlawSearch::get_min_h_batch_split() {
    // Handle flaws of refined abstract state
    if (last_refined_abstract_state_id != -1) {
        vector<State> states_to_handle =
            move(flawed_states.at(last_refined_abstract_state_id));
        flawed_states.erase(last_refined_abstract_state_id);
        for (const State &s : states_to_handle) {
            if (task_properties::is_goal_state(task_proxy, s)) {
                return nullptr;
            }
            int abs_id = get_abstract_state_id(s);
            if (get_h_value(abs_id) == best_flaw_h) {
                add_flaw(abs_id, s);
            }
        }
    }

    auto search_status = SearchStatus::FAILED;
    if (flawed_states.empty()) {
        search_status = search_for_flaws();
    }

    // Memory padding
    if (search_status == TIMEOUT)
        return nullptr;

    // Flaws to refine are present
    if (search_status == FAILED) {
        assert(!flawed_states.empty());

        ++num_overall_refined_flaws;

        int abstract_state_id = flawed_states.begin()->first;

        unique_ptr<Split> split;
        if (pick_flaw == PickFlaw::MIN_H_BATCH_MULTI_SPLIT) {
            split = create_split(flawed_states.begin()->second,
                                 abstract_state_id);
        } else {
            const State &state = *flawed_states[abstract_state_id].begin();
            split = create_split({state}, abstract_state_id);
        }

        return split;
    }

    assert(search_status == SOLVED);
    return nullptr;
}

FlawSearch::FlawSearch(const shared_ptr<AbstractTask> &task,
                       const vector<int> &domain_sizes,
                       const Abstraction &abstraction,
                       const ShortestPaths &shortest_paths,
                       utils::RandomNumberGenerator &rng,
                       PickFlaw pick_flaw,
                       PickSplit pick_split,
                       bool debug) :
    task_proxy(*task),
    domain_sizes(domain_sizes),
    abstraction(abstraction),
    shortest_paths(shortest_paths),
    split_selector(task, pick_split, debug),
    rng(rng),
    pick_flaw(pick_flaw),
    debug(debug),
    open_list(nullptr),
    state_registry(utils::make_unique_ptr<StateRegistry>(task_proxy)),
    search_space(nullptr),
    statistics(nullptr),
    last_refined_abstract_state_id(-1),
    best_flaw_h((pick_flaw == PickFlaw::MAX_H_SINGLE) ? -INF : INF),
    num_searches(0),
    num_overall_refined_flaws(0),
    num_overall_expanded_concrete_states(0) {
    shared_ptr<Evaluator> g_evaluator = make_shared<g_evaluator::GEvaluator>();
    Options options;
    options.set("eval", g_evaluator);
    options.set("pref_only", false);

    open_list =
        make_shared<standard_scalar_open_list::BestFirstOpenListFactory>
            (options)->create_state_open_list();
    timer.stop();
    timer.reset();
}

unique_ptr<Split> FlawSearch::get_split() {
    timer.resume();
    unique_ptr<Split> split = nullptr;

    switch (pick_flaw) {
    case PickFlaw::RANDOM_H_SINGLE:
        split = get_random_single_split();
        break;
    case PickFlaw::MIN_H_SINGLE:
        split = get_single_split();
        break;
    case PickFlaw::MAX_H_SINGLE:
        split = get_single_split();
        break;
    case PickFlaw::MIN_H_BATCH:
        split = get_min_h_batch_split();
        break;
    case PickFlaw::MIN_H_BATCH_MULTI_SPLIT:
        split = get_min_h_batch_split();
        break;
    default:
        utils::g_log << "Invalid pick flaw strategy: " << static_cast<int>(pick_flaw)
                     << endl;
        utils::exit_with(utils::ExitCode::SEARCH_INPUT_ERROR);
    }

    if (split) {
        last_refined_abstract_state_id = split->abstract_state_id;
    }
    timer.stop();
    return split;
}

void FlawSearch::print_statistics() const {
    utils::g_log << endl;
    utils::g_log << "#Flaw searches: " << num_searches << endl;
    utils::g_log << "#Flaws refined: " << num_overall_refined_flaws << endl;
    utils::g_log << "#Expanded concrete states: "
                 << num_overall_expanded_concrete_states << endl;
    utils::g_log << "Flaw search time: " << timer << endl;
    utils::g_log << "Avg flaws refined: "
                 << num_overall_refined_flaws / (float)num_searches << endl;
    utils::g_log << "Avg expanded concrete states: "
                 << num_overall_expanded_concrete_states / (float)num_searches
                 << endl;
    utils::g_log << "Avg Flaw search time: "
                 << timer() / (float)num_searches << endl;
    utils::g_log << endl;
}
}
