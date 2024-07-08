#include "flaw_search.h"
#include "abstraction.h"
#include "split_selector.h"
#include "transition_system.h"

namespace cartesian_abstractions {
using namespace std;

std::unique_ptr<Split> FlawSearch::splits_cache_get(const LegacyFlaw &f,
                                                    Cost solution_cost,
                                                    bool backward_direction,
                                                    bool split_unwanted_values) {
    OptimalTransitions opt_tr;
    if (backward_direction) {
        opt_tr = get_f_optimal_backward_transitions(f.abstract_state_id);
    } else {
        opt_tr = get_f_optimal_transitions(f.abstract_state_id);
    }
    // Check split is cached and f-optimal/backward f-optimal transitions have not changed.
    if (splits_cache.count(f.abstract_state_id) == 0 ||
        splits_cache[f.abstract_state_id].count(backward_direction) == 0 ||
        splits_cache[f.abstract_state_id][backward_direction].count(split_unwanted_values) == 0 ||
        splits_cache[f.abstract_state_id][backward_direction][split_unwanted_values].count(f) == 0 ||
        opt_tr_cache[f.abstract_state_id].at(backward_direction) != opt_tr) {
        splits_cache[f.abstract_state_id][backward_direction][split_unwanted_values].erase(f);
        splits_cache[f.abstract_state_id][backward_direction][split_unwanted_values].emplace(
            f, create_split_from_flaw(f, solution_cost, backward_direction, split_unwanted_values));
        opt_tr_cache[f.abstract_state_id].erase(backward_direction);
        opt_tr_cache[f.abstract_state_id].emplace(backward_direction, std::move(opt_tr));
    }
    auto split = splits_cache[f.abstract_state_id][backward_direction][split_unwanted_values].at(f);
    return utils::make_unique_ptr<Split>(
        split->abstract_state_id, split->var_id, split->value,
        vector<int>(split->values), split->count, split->op_cost);
}

void FlawSearch::splits_cache_invalidate(int abstract_state_id) {
    if (!splits_cache.empty()) {
        splits_cache.erase(abstract_state_id);
        // Invalidate cache of flaws with incoming/outgoing
        // transitions to this state. f-optimal/f-backward-optimal only are not
        // enough, all transitions must be invalidated.
        // Flaws in init/goal states are not necessary
        // to be invalidated, but detecting them is more expensive and they are
        // a low percentage of flaws.
        for (auto &&tr : abstraction.get_transition_system().get_incoming_transitions()[abstract_state_id]) {
            if (splits_cache.count(tr.target_id) > 0 && splits_cache[tr.target_id].count(false) > 0) {
                splits_cache[tr.target_id].erase(false);
                opt_tr_cache[tr.target_id].erase(false);
            }
        }
        for (auto &&tr : abstraction.get_transition_system().get_outgoing_transitions()[abstract_state_id]) {
            if (splits_cache.count(tr.target_id) > 0 && splits_cache[tr.target_id].count(true) > 0) {
                splits_cache[tr.target_id].erase(true);
                opt_tr_cache[tr.target_id].erase(true);
            }
        }
    }
}
} // namespace cegar
