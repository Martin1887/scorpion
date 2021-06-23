#include "random_pattern.h"

#include "../option_parser.h"

#include "../task_proxy.h"

#include "../utils/countdown_timer.h"
#include "../utils/logging.h"
#include "../utils/rng.h"

#include <algorithm>
#include <unordered_set>

using namespace std;

namespace pdbs{
static bool time_limit_reached(
    const utils::CountdownTimer &timer, utils::Verbosity verbosity) {
    if (timer.is_expired()) {
        if (verbosity >= utils::Verbosity::NORMAL) {
            utils::g_log << "time limit reached." << endl;
        }
        return true;
    }
    return false;
}

Pattern generate_random_pattern(
    int max_pdb_size,
    double max_time,
    utils::Verbosity verbosity,
    const shared_ptr<utils::RandomNumberGenerator> &rng,
    const TaskProxy &task_proxy,
    int goal_variable,
    vector<vector<int>> &cg_neighbors) {
    utils::CountdownTimer timer(max_time);
    int current_var = goal_variable;
    unordered_set<int> visited_vars;
    visited_vars.insert(current_var);
    VariablesProxy variables = task_proxy.get_variables();
    int pdb_size = variables[current_var].get_domain_size();
    while (!time_limit_reached(timer, verbosity)) {
        // Pick random cg neighbor.
        rng->shuffle(cg_neighbors[current_var]);
        int neighbor_var = -1;
        for (int candidate : cg_neighbors[current_var]) {
            if (!visited_vars.count(candidate)) {
                neighbor_var = candidate;
                break;
            }
        }

        if (neighbor_var != -1 && utils::is_product_within_limit(
            pdb_size, variables[neighbor_var].get_domain_size(), max_pdb_size)) {
            pdb_size *= variables[neighbor_var].get_domain_size();
            visited_vars.insert(neighbor_var);
            current_var = neighbor_var;
        } else {
            break;
        }
    }

    Pattern pattern(visited_vars.begin(), visited_vars.end());
    sort(pattern.begin(), pattern.end());
    return pattern;
}

void add_random_pattern_bidirectional_option_to_parser(options::OptionParser &parser) {
    parser.add_option<bool>(
        "bidirectional",
        "this option decides if the causal graph is considered to be "
        "directed or undirected selecting predecessors of already selected "
        "variables. If true (default), it is considered to be undirected "
        "(precondition-effect edges are bidirectional). If false, it is "
        "considered to be directed (a variable is a neighbor only if it is a "
        "predecessor.",
        "true");
}
}
