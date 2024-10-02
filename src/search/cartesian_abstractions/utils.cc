#include "utils.h"

#include "abstract_state.h"
#include "abstraction.h"
#include "flaw_search.h"
#include "split_selector.h"
#include "transition.h"
#include "transition_system.h"

#include "../lp/lp_solver.h"
#include "../plugins/plugin.h"
#include "../heuristics/additive_heuristic.h"
#include "../task_utils/ac3_disambiguation.h"
#include "../task_utils/cartesian_set_facts_proxy_iterator.h"
#include "../task_utils/disambiguated_operator.h"
#include "../task_utils/disambiguation_method.h"
#include "../task_utils/task_properties.h"
#include "../utils/logging.h"
#include "../utils/memory.h"
#include "../utils/rng_options.h"

#include <algorithm>
#include <cassert>
#include <fstream>
#include <iostream>
#include <map>

using namespace std;
using namespace disambiguation;

namespace cartesian_abstractions {
class SubtaskGenerator;

unique_ptr<additive_heuristic::AdditiveHeuristic> create_additive_heuristic(
    const shared_ptr<AbstractTask> &task) {
    plugins::Options opts;
    opts.set<shared_ptr<AbstractTask>>("transform", task);
    opts.set<bool>("cache_estimates", false);
    opts.set<utils::Verbosity>("verbosity", utils::Verbosity::SILENT);
    return make_unique<additive_heuristic::AdditiveHeuristic>(opts);
}

static bool operator_applicable(
    const DisambiguatedOperator &op, const std::vector<utils::HashSet<int>> &facts) {
    const CartesianSet &pre = op.get_precondition().get_cartesian_set();
    int var = -1;

    for (auto &var_facts : facts) {
        var++;
        if (pre.all_values_set(var)) {
            continue;
        }
        bool some_exists = false;
        for (auto &&[fact_var, fact_value] : pre.iter(var)) {
            if (var_facts.count(fact_value) == 1) {
                some_exists = true;
                break;
            }
        }
        if (!some_exists) {
            return false;
        }
    }
    return true;
}

static std::vector<utils::HashSet<int>> compute_possibly_before_facts(
    const shared_ptr<vector<DisambiguatedOperator>> &ops,
    const TaskProxy &task,
    const FactProxy &last_fact) {
    std::vector<utils::HashSet<int>> pb_facts{task.get_variables().size(), utils::HashSet<int>()};

    // Add facts from initial state.
    for (FactProxy fact : task.get_initial_state())
        pb_facts[fact.get_pair().var].insert(fact.get_pair().value);

    bool updated = true;
    int last_fact_var = last_fact.get_pair().var;
    int last_fact_value = last_fact.get_pair().value;
    int n_vars = task.get_variables().size();

    while (updated) {
        updated = false;
        for (DisambiguatedOperator &op : *ops) {
            const CartesianSet &post = op.get_post().get_cartesian_set();
            // Ignore operators that achieve last_fact.
            if (op.has_effect(last_fact_var) && post.test(last_fact_var, last_fact_value)) {
                continue;
            }
            // Add all facts that are achieved by an applicable operator.
            if (operator_applicable(op, pb_facts)) {
                for (int var = 0; var < n_vars; var++) {
                    int effect = op.get_effect(var);
                    if (effect != -1) {
                        if (pb_facts[var].insert(effect).second) {
                            updated = true;
                        }
                    }
                }
            }
        }
    }
    return pb_facts;
}

std::vector<utils::HashSet<int>> get_relaxed_possible_before(
    const shared_ptr<vector<DisambiguatedOperator>> &ops,
    const TaskProxy &task,
    const FactProxy &fact) {
    std::vector<utils::HashSet<int>> reachable_facts =
        compute_possibly_before_facts(ops, task, fact);
    reachable_facts[fact.get_pair().var].insert(fact.get_pair().value);
    return reachable_facts;
}

vector<int> get_domain_sizes(const TaskProxy &task) {
    vector<int> domain_sizes;
    for (VariableProxy var : task.get_variables())
        domain_sizes.push_back(var.get_domain_size());
    return domain_sizes;
}

static void add_memory_padding_option(plugins::Feature &feature) {
    feature.add_option<int>(
        "memory_padding",
        "amount of extra memory in MB to reserve for recovering from "
        "out-of-memory situations gracefully. When the memory runs out, we "
        "stop refining and start the search. Due to memory fragmentation, "
        "the memory used for building the abstraction (states, transitions, "
        "etc.) often can't be reused for things that require big continuous "
        "blocks of memory. It is for this reason that we require a rather "
        "large amount of memory padding by default.",
        "500",
        plugins::Bounds("0", "infinity"));
}

static void add_dot_graph_verbosity(plugins::Feature &feature) {
    feature.add_option<DotGraphVerbosity>(
        "dot_graph_verbosity",
        "verbosity of printing/writing dot graphs",
        "silent");
}

string create_dot_graph(const TaskProxy &task_proxy, const Abstraction &abstraction) {
    ostringstream oss;
    int num_states = abstraction.get_num_states();
    oss << "digraph transition_system";
    oss << " {" << endl;
    oss << "    node [shape = none] start;" << endl;
    for (int i = 0; i < num_states; ++i) {
        bool is_init = (i == abstraction.get_initial_state().get_id());
        bool is_goal = abstraction.get_goals().count(i);
        oss << "    node [shape = " << (is_goal ? "doublecircle" : "circle")
            << "] " << i << ";" << endl;
        if (is_init)
            oss << "    start -> " << i << ";" << endl;
    }
    for (int state_id = 0; state_id < num_states; ++state_id) {
        map<int, vector<int>> parallel_transitions;
        auto transitions =
            abstraction.get_transition_system().get_outgoing_transitions();
        for (const Transition &t : transitions[state_id]) {
            parallel_transitions[t.target_id].push_back(t.op_id);
        }
        for (auto &pair : parallel_transitions) {
            int target = pair.first;
            vector<int> &operators = pair.second;
            sort(operators.begin(), operators.end());
            vector<string> operator_names;
            operator_names.reserve(operators.size());
            for (int op_id : operators) {
                operator_names.push_back(task_proxy.get_operators()[op_id].get_name());
            }
            oss << "    " << state_id << " -> " << target << " [label = \""
                << utils::join(operator_names, ", ") << "\"];" << endl;
        }
    }
    oss << "}" << endl;
    return oss.str();
}

void write_to_file(const string &file_name, const string &content) {
    ofstream output_file(file_name);
    if (output_file.is_open()) {
        output_file << content;
        output_file.close();
    } else {
        ABORT("failed to open " + file_name);
    }
    if (output_file.fail()) {
        ABORT("failed to write to " + file_name);
    }
}

void add_common_cegar_options(plugins::Feature &feature) {
    feature.add_list_option<shared_ptr<SubtaskGenerator>>(
        "subtasks",
        "subtask generators",
        "[landmarks(order=random), goals(order=random)]");
    feature.add_option<int>(
        "max_states",
        "maximum sum of abstract states over all abstractions",
        "infinity",
        plugins::Bounds("1", "infinity"));
    feature.add_option<int>(
        "max_transitions",
        "maximum sum of state-changing transitions (excluding self-loops) over "
        "all abstractions",
        "1M",
        plugins::Bounds("0", "infinity"));
    feature.add_option<double>(
        "max_time",
        "maximum time in seconds for building abstractions",
        "infinity",
        plugins::Bounds("0.0", "infinity"));

    add_memory_padding_option(feature);
    add_dot_graph_verbosity(feature);
    utils::add_rng_options(feature);

    feature.add_option<int>(
        "max_concrete_states_per_abstract_state",
        "maximum number of flawed concrete states stored per abstract state",
        "infinity",
        plugins::Bounds("1", "infinity"));
    feature.add_option<int>(
        "max_state_expansions",
        "maximum number of state expansions per flaw search",
        "1M",
        plugins::Bounds("1", "infinity"));

    feature.add_option<bool>(
        "print_h_distribution",
        "print h distribution in the concrete space",
        "false");

    feature.add_option<bool>(
        "print_useless_refinements",
        "print useless refinements at the end of the abstraction",
        "false");

    lp::add_lp_solver_option_to_feature(feature);

    feature.add_option<shared_ptr<DisambiguationMethod>>(
        "operators_disambiguation",
        "method to disambiguate preconditions and effects of operators",
        "none()");
    feature.add_option<shared_ptr<DisambiguationMethod>>(
        "abstract_space_disambiguation",
        "method to disambiguate abstract states",
        "none()");
    feature.add_option<shared_ptr<DisambiguationMethod>>(
        "flaw_search_states_disambiguation",
        "method to disambiguate partial states obtained during the flaws search",
        "none()");
}

static plugins::TypedEnumPlugin<DotGraphVerbosity> _enum_plugin({
        {"silent", ""},
        {"write_to_console", ""},
        {"write_to_file", ""}
    });
}
