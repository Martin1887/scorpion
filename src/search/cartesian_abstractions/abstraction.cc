#include "abstraction.h"

#include "abstract_state.h"
#include "refinement_hierarchy.h"
#include "transition.h"
#include "transition_system.h"
#include "utils.h"

#include "../task_utils/disambiguation_method.h"
#include "../task_utils/task_properties.h"
#include "../utils/logging.h"
#include "../utils/math.h"
#include "../utils/memory.h"

#include <algorithm>
#include <cassert>
#include <iostream>
#include <unordered_map>

using namespace std;

namespace cartesian_abstractions {
Abstraction::Abstraction(const shared_ptr<AbstractTask> &task,
                         const shared_ptr<vector<disambiguation::DisambiguatedOperator>> &operators,
                         shared_ptr<MutexInformation> &mutex_information,
                         shared_ptr<disambiguation::DisambiguationMethod> &abstract_space_disambiguation,
                         utils::LogProxy &log)
    : task_proxy(TaskProxy(*task)),
      transition_system(make_unique<TransitionSystem>(operators)),
      concrete_initial_state(task_proxy.get_initial_state()),
      goal_facts(task_properties::get_fact_pairs(task_proxy.get_goals())),
      mutex_information(mutex_information),
      abstract_space_disambiguation(abstract_space_disambiguation),
      refinement_hierarchy(utils::make_unique_ptr<RefinementHierarchy>(task)),
      log(log) {
    initialize_trivial_abstraction(get_domain_sizes(TaskProxy(*task)));
    bool disambiguated = disambiguate_state(0);
    transition_system->add_loops_in_trivial_abstraction(*states[0], disambiguated);
}

Abstraction::~Abstraction() {
}

bool Abstraction::disambiguate_state(int state_id) {
    return disambiguate_state(*states[state_id]);
}

bool Abstraction::disambiguate_state(AbstractState &state) {
    bool disambiguated = abstract_space_disambiguation->disambiguate(state, *mutex_information);
    if (disambiguated) {
        n_disambiguations++;
    }
    return disambiguated;
}

const AbstractState &Abstraction::get_initial_state() const {
    return *states[init_id];
}

int Abstraction::get_num_states() const {
    return states.size();
}

const Goals &Abstraction::get_goals() const {
    return goals;
}

const AbstractState &Abstraction::get_state(int state_id) const {
    return *states[state_id];
}

int Abstraction::get_abstract_state_id(const State &state) const {
    int node_id = refinement_hierarchy->get_node_id(state);
    return refinement_hierarchy->nodes.at(node_id).get_state_id();
}

const TransitionSystem &Abstraction::get_transition_system() const {
    return *transition_system;
}

unique_ptr<RefinementHierarchy> Abstraction::extract_refinement_hierarchy() {
    assert(refinement_hierarchy);
    return move(refinement_hierarchy);
}

void Abstraction::mark_all_goal_states_as_goals() {
    if (log.is_at_least_debug()) {
        log << "Mark all goal states as goals." << endl;
    }
    goals.clear();
    for (const auto &state : states) {
        if (state->includes(goal_facts)) {
            goals.insert(state->get_id());
        }
    }
}

void Abstraction::initialize_trivial_abstraction(const vector<int> &domain_sizes) {
    unique_ptr<AbstractState> init_state =
        AbstractState::get_trivial_abstract_state(domain_sizes);
    init_id = init_state->get_id();
    goals.insert(init_state->get_id());
    states.push_back(move(init_state));
}

AbstractStateSplit Abstraction::split(
    const AbstractState &state, int var, const std::vector<int> &wanted) const {
    int v_id = state.get_id();
    // Reuse state ID from obsolete parent to obtain consecutive IDs.
    int v1_id = v_id;
    int v2_id = get_num_states();

    pair<CartesianSet, CartesianSet> cartesian_sets =
        state.split_domain(var, wanted);
    CartesianSet &v1_cartesian_set = cartesian_sets.first;
    CartesianSet &v2_cartesian_set = cartesian_sets.second;

    vector<int> v2_values = wanted;
    assert(v2_values == v2_cartesian_set.get_values(var));
    // We partition the abstract domain into two subsets. Since the refinement
    // hierarchy stores helper nodes for all values of one of the children, we
    // prefer to use the smaller subset.
    if (v2_values.size() > 1) { // Quickly test necessary condition.
        vector<int> v1_values = v1_cartesian_set.get_values(var);
        if (v2_values.size() > v1_values.size()) {
            swap(v1_id, v2_id);
            swap(v1_values, v2_values);
            swap(v1_cartesian_set, v2_cartesian_set);
        }
    }

    // Ensure that the initial state always has state ID 0.
    if ((v1_id == init_id &&
         v2_cartesian_set.test(var, concrete_initial_state[var].get_value()))
        || (v2_id == init_id &&
            v1_cartesian_set.test(var, concrete_initial_state[var].get_value()))) {
        swap(v1_id, v2_id);
    }

    return AbstractStateSplit {v1_id, v2_id, v2_values, v1_cartesian_set, v2_cartesian_set};
}

tuple<int, int, bool, Transitions, Transitions> Abstraction::refine(
    const AbstractState &state, int var, const vector<int> &wanted) {
    if (log.is_at_least_debug())
        log << "Refine " << state << " for " << var << "=" << wanted << endl;

    int v_id = state.get_id();
    auto split_result = split(state, var, wanted);

    // Update refinement hierarchy.
    pair<NodeID, NodeID> node_ids = refinement_hierarchy->split(
        state.get_node_id(), var,
        split_result.v2_values, split_result.v1_id, split_result.v2_id);

    unique_ptr<AbstractState> v1 = make_unique<AbstractState>(
        split_result.v1_id, node_ids.first, move(split_result.v1_cartesian_set));
    unique_ptr<AbstractState> v2 = make_unique<AbstractState>(
        split_result.v2_id, node_ids.second, move(split_result.v2_cartesian_set));
    assert(state.includes(*v1));
    assert(state.includes(*v2));

    vector<int> modified_vars{};
    bool wanted_in_v1 = true;
    if (!v1->includes(var, wanted[0])) {
        wanted_in_v1 = false;
    }
    bool disambiguated = disambiguate_state(*v1) || disambiguate_state(*v2);
    if (disambiguated) {
        if (v1->got_empty()) {
            n_removed_states++;
        }
        if (v2->got_empty()) {
            n_removed_states++;
        }
        const CartesianSet &v_set = state.get_cartesian_set();
        const CartesianSet &v1_set = v1->get_cartesian_set();
        const CartesianSet &v2_set = v2->get_cartesian_set();
        int n_vars = v_set.get_n_vars();
        for (int analysed_var = 0; analysed_var < n_vars; analysed_var++) {
            if (!v_set.is_equal_in_var(v1_set, var) || !v_set.is_equal_in_var(v2_set, var)) {
                modified_vars.push_back(analysed_var);
                // A node must be created in refinement hierarchy for the
                // values that are not included in any abstract state.
                if (var == analysed_var) {
                    int wanted_size = wanted.size();
                    if (wanted_in_v1) {
                        if (v1_set.count(analysed_var) != wanted_size) {
                            // Some wanted value has been disambiguated, split needed.
                            pair<NodeID, NodeID> new_node_ids = refinement_hierarchy->split(
                                v1->get_node_id(), analysed_var,
                                v1_set.get_values(analysed_var), NO_ABSTRACT_STATE, v1->get_id());
                            v1->set_node_id(new_node_ids.second);
                        }
                        if (v2_set.count(analysed_var) != v_set.count(analysed_var) - wanted_size) {
                            // Some wanted value has been disambiguated, split needed.
                            pair<NodeID, NodeID> new_node_ids = refinement_hierarchy->split(
                                v2->get_node_id(), analysed_var,
                                v2_set.get_values(analysed_var), NO_ABSTRACT_STATE, v2->get_id());
                            v2->set_node_id(new_node_ids.second);
                        }
                    } else {
                        if (v2_set.count(analysed_var) != wanted_size) {
                            // Some wanted value has been disambiguated, split needed.
                            pair<NodeID, NodeID> new_node_ids = refinement_hierarchy->split(
                                v2->get_node_id(), analysed_var,
                                v2_set.get_values(analysed_var), NO_ABSTRACT_STATE, v2->get_id());
                            v2->set_node_id(new_node_ids.second);
                        }
                        if (v1_set.count(analysed_var) != v_set.count(analysed_var) - wanted_size) {
                            // Some wanted value has been disambiguated, split needed.
                            pair<NodeID, NodeID> new_node_ids = refinement_hierarchy->split(
                                v1->get_node_id(), analysed_var,
                                v1_set.get_values(analysed_var), NO_ABSTRACT_STATE, v1->get_id());
                            v1->set_node_id(new_node_ids.second);
                        }
                    }
                } else {
                    // States must be split for sure if their size is different in the var.
                    int parent_size = v_set.count(analysed_var);
                    if (v1_set.count(analysed_var) != parent_size) {
                        pair<NodeID, NodeID> new_node_ids = refinement_hierarchy->split(
                            v1->get_node_id(), analysed_var,
                            v1_set.get_values(analysed_var), NO_ABSTRACT_STATE, v1->get_id());
                        v1->set_node_id(new_node_ids.second);
                    }
                    if (v2_set.count(analysed_var) != parent_size) {
                        pair<NodeID, NodeID> new_node_ids = refinement_hierarchy->split(
                            v2->get_node_id(), analysed_var,
                            v2_set.get_values(analysed_var), NO_ABSTRACT_STATE, v2->get_id());
                        v2->set_node_id(new_node_ids.second);
                    }
                }
            }
        }
    } else {
        modified_vars.push_back(var);
    }

    if (goals.count(v_id)) {
        goals.erase(v_id);
        if (v1->includes(goal_facts)) {
            goals.insert(split_result.v1_id);
        }
        if (v2->includes(goal_facts)) {
            goals.insert(split_result.v2_id);
        }
        if (log.is_at_least_debug()) {
            log << "Goal states: " << goals.size() << endl;
        }
    }

    tuple<Transitions, Transitions> old_incoming_outgoing =
        transition_system->rewire(states, v_id, *v1, *v2, modified_vars);

    states.emplace_back();
    states[split_result.v1_id] = move(v1);
    states[split_result.v2_id] = move(v2);

    if (log.is_at_least_debug()) {
        for (int goal : goals) {
            log << *states[goal] << endl;
        }
    }

    assert(init_id == 0);
    assert(get_initial_state().includes(concrete_initial_state));

    return {split_result.v1_id, split_result.v2_id, disambiguated,
            get<0>(old_incoming_outgoing), get<1>(old_incoming_outgoing)};
}

SimulatedRefinement Abstraction::simulate_refinement(
    shared_ptr<TransitionSystem> &simulated_transition_system,
    const AbstractState &state,
    int var,
    const std::vector<int> &wanted) const {
    if (log.is_at_least_debug())
        log << "Simulate refinement " << state << " for "
            << var << "=" << wanted << endl;

    int v_id = state.get_id();

    auto split_result = split(state, var, wanted);

    // Node ids are not used in simulated refinements.
    unique_ptr<AbstractState> v1 = make_unique<AbstractState>(
        split_result.v1_id, split_result.v1_id, move(split_result.v1_cartesian_set));
    unique_ptr<AbstractState> v2 = make_unique<AbstractState>(
        split_result.v2_id, split_result.v2_id, move(split_result.v2_cartesian_set));
    assert(state.includes(*v1));
    assert(state.includes(*v2));

    simulated_transition_system->force_new_transitions(get_transition_system().get_incoming_transitions(),
                                                       get_transition_system().get_outgoing_transitions(),
                                                       get_transition_system().get_loops());

    // disambiguate_state function is not called because statistics must not be
    // increased for simulated refinements.
    bool disambiguated = abstract_space_disambiguation->disambiguate(*v1, *mutex_information) ||
        abstract_space_disambiguation->disambiguate(*v2, *mutex_information);
    vector<int> modified_vars{};
    if (disambiguated) {
        const CartesianSet &v_set = state.get_cartesian_set();
        const CartesianSet &v1_set = v1->get_cartesian_set();
        const CartesianSet &v2_set = v2->get_cartesian_set();
        int n_vars = v_set.get_n_vars();
        for (int var = 0; var < n_vars; var++) {
            if (!v_set.is_equal_in_var(v1_set, var) || !v_set.is_equal_in_var(v2_set, var)) {
                modified_vars.push_back(var);
            }
        }
    } else {
        modified_vars.push_back(var);
    }
    SimulatedRefinement ref(simulated_transition_system,
                            goals,
                            split_result.v1_id,
                            split_result.v2_id,
                            disambiguated,
                            get_transition_system().get_incoming_transitions()[v_id],
                            get_transition_system().get_outgoing_transitions()[v_id]);

    if (ref.goals.count(v_id)) {
        ref.goals.erase(v_id);
        if (v1->includes(goal_facts)) {
            ref.goals.insert(split_result.v1_id);
        }
        if (v2->includes(goal_facts)) {
            ref.goals.insert(split_result.v2_id);
        }
        if (log.is_at_least_debug()) {
            log << "Goal states: " << ref.goals.size() << endl;
        }
    }

    ref.transition_system->rewire(states, v_id, *v1, *v2, modified_vars, true);

    assert(init_id == 0);
    assert(get_initial_state().includes(concrete_initial_state));

    return ref;
}

void Abstraction::print_statistics() const {
    if (log.is_at_least_normal()) {
        log << "States: " << get_num_states() << endl;
        log << "Goal states: " << goals.size() << endl;
        transition_system->print_statistics(log);
        log << "Nodes in refinement hierarchy: "
            << refinement_hierarchy->get_num_nodes() << endl;
        log << "Disambiguated states: " << n_disambiguations << endl;
        log << "Removed states: " << n_removed_states << endl;
    }
}

void Abstraction::dump() const {
    auto outgoing = transition_system->get_outgoing_transitions();
    for (int i = 0; i < get_num_states(); i++) {
        cout << "State " << i << ":" << endl;
        cout << "    " << get_state(i) << endl;
        cout << "    Outgoing transitions:" << endl;
        for (Transition out : outgoing[i]) {
            OperatorProxy op = task_proxy.get_operators()[out.op_id];
            cout << "        " << op.get_name() << " " << out.target_id << endl;
        }
    }
}

void Abstraction::h_distribution(const vector<int> &goal_distances,
                                 const vector<int> &init_distances) const {
    // For each h value, the number of states with it as h value.
    utils::HashMap<int, vector<vector<int>>> h_distribution{};
    int n_abstract_states = get_num_states();
    vector<vector<int>> unreachable_states{};

    for (int i = 0; i < n_abstract_states; i++) {
        vector<int> n_states = get_state(i).count();
        int h = goal_distances[i];
        vector<vector<int>> current = h_distribution[h];
        current.push_back(n_states);
        h_distribution[h] = current;
        if (init_distances[i] == INF) {
            unreachable_states.push_back(n_states);
        }
    }

    log << "Total number of concrete states: " << get_domain_sizes(task_proxy)
        << endl;

    log << "Number of unreachable concrete states: " << unreachable_states << endl;
    for (auto h_nstates : h_distribution) {
        log << "Distribution of h, h=" << h_nstates.first << " for "
            << h_nstates.second << " concrete states" << endl;
    }
}
}
