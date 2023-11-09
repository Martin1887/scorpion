#include "flaw_search.h"

namespace cegar {
SplitAndAbsState FlawSearch::splits_cache_get(LegacyFlaw f,
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
        splits_cache[f.abstract_state_id][backward_direction][split_unwanted_values].emplace(
            f, SplitAndAbsState(get_split_from_flaw(std::move(f), backward_direction, split_unwanted_values)));
        opt_tr_cache[f.abstract_state_id].emplace(backward_direction, std::move(opt_tr));
    }
    SplitAndAbsState *value = &splits_cache[f.abstract_state_id][backward_direction][split_unwanted_values].at(f);
    return SplitAndAbsState {utils::make_unique_ptr<Split>(*value->split), value->abs};
}

void FlawSearch::splits_cache_invalidate(int abstract_state_id) {
    splits_cache.erase(abstract_state_id);
    // Invalidate cache of flaws with f-optimal/backward f-optimal
    // transitions to this state. Flaws in init/goal states are not necessary
    // to be invalidated, but detecting them is more expensive and they are a
    // low percentage of flaws.
    for (auto &&tr : get_f_optimal_transitions(abstract_state_id, true)) {
        for (int id : tr.second) {
            if (splits_cache.count(id) > 0 && splits_cache[id].count(false) > 0) {
                splits_cache[id].erase(false);
                opt_tr_cache[id].erase(false);
            }
        }
    }
    for (auto &&tr : get_f_optimal_backward_transitions(abstract_state_id, true)) {
        for (int id : tr.second) {
            if (splits_cache.count(id) > 0 && splits_cache[id].count(true) > 0) {
                splits_cache[id].erase(true);
                opt_tr_cache[id].erase(true);
            }
        }
    }
}
}
