#include "flaw_search.h"

#include "abstraction.h"
#include "abstract_state.h"
#include "flaw.h"
#include "shortest_paths.h"
#include "split_selector.h"
#include "transition_system.h"
#include "utils.h"

#include "../task_utils/successor_generator.h"
#include "../task_utils/task_properties.h"
#include "../utils/countdown_timer.h"
#include "../utils/rng.h"

#include <iterator>
#include <locale>

using namespace std;

namespace cegar {
void FlawSearch::get_deviation_splits(
    const AbstractState &abs_state,
    const vector<State> &conc_states,
    const vector<int> &unaffected_variables,
    const AbstractState &target_abs_state,
    const vector<int> &domain_sizes,
    vector<vector<Split>> &splits,
    bool split_unwanted_values) {
    /*
      For each fact in the concrete state that is not contained in the
      target abstract state, loop over all values in the domain of the
      corresponding variable. The values that are in both the current and
      the target abstract state are the "wanted" ones, i.e., the ones that
      we want to split off. This test can be specialized for applicability and
      deviation flaws. Here, we consider deviation flaws.

      Let the desired abstract transition be (a, o, t) and the deviation be
      (a, o, b). We distinguish three cases for each variable v:

      pre(o)[v] defined: no split possible since o is applicable in s.
      pre(o)[v] undefined, eff(o)[v] defined: no split possible since regression adds whole domain.
      pre(o)[v] and eff(o)[v] undefined: if s[v] \notin t[v], wanted = intersect(a[v], b[v]).
    */
    // Note: it could be faster to use an efficient hash map for this.
    vector<vector<int>> fact_count(domain_sizes.size());
    for (size_t var = 0; var < domain_sizes.size(); ++var) {
        fact_count[var].resize(domain_sizes[var], 0);
    }
    for (const State &conc_state : conc_states) {
        for (int var : unaffected_variables) {
            int state_value = conc_state[var].get_value();
            ++fact_count[var][state_value];
        }
    }
    for (size_t var = 0; var < domain_sizes.size(); ++var) {
        for (int value = 0; value < domain_sizes[var]; ++value) {
            if (fact_count[var][value] && !target_abs_state.contains(var, value)) {
                // Note: we could precompute the "wanted" vector, but not the split.
                vector<int> wanted;
                for (int value = 0; value < domain_sizes[var]; ++value) {
                    if (abs_state.contains(var, value) &&
                        target_abs_state.contains(var, value)) {
                        wanted.push_back(value);
                    }
                }
                assert(!wanted.empty());
                if (split_unwanted_values) {
                    for (int want : wanted) {
                        FlawSearch::add_split(splits, Split(
                                                  abs_state.get_id(), var, want, {value},
                                                  fact_count[var][value]), true);
                    }
                } else {
                    FlawSearch::add_split(splits, Split(
                                              abs_state.get_id(), var, value, move(wanted),
                                              fact_count[var][value]));
                }
            }
        }
    }
}

unique_ptr<Split> FlawSearch::get_split_from_flaw(const LegacyFlaw &flaw,
                                                  const bool backward,
                                                  const bool split_unwanted_values) {
    if (backward) {
        if (flaw.split_last_state) {
            return create_backward_split_from_init_state({flaw.flaw_search_state},
                                                         flaw.abstract_state_id,
                                                         split_unwanted_values);
        } else {
            return create_backward_split({flaw.flaw_search_state},
                                         flaw.abstract_state_id,
                                         split_unwanted_values);
        }
    } else {
        if (flaw.split_last_state) {
            return create_split_from_goal_state({flaw.flaw_search_state},
                                                flaw.abstract_state_id,
                                                split_unwanted_values);
        } else {
            return create_split({flaw.flaw_search_state},
                                flaw.abstract_state_id,
                                split_unwanted_values);
        }
    }
}

SplitProperties FlawSearch::get_split_legacy(const Solution &solution,
                                             const bool backward,
                                             const bool split_unwanted_values) {
    if (log.is_at_least_debug()) {
        log << "Abstraction: " << endl;
        abstraction.dump();
    }
    unique_ptr<pair<StateID, int>> split_pair;
    if (backward) {
        unique_ptr<LegacyFlaw> backward_flaw = get_flaw_legacy_backward(solution);
        if (backward_flaw) {
            return SplitProperties(get_split_from_flaw(*backward_flaw, true, split_unwanted_values),
                                   get_plan_perc(backward_flaw->abstract_state_id, solution), true, 0, 1);
        }
    } else {
        unique_ptr<LegacyFlaw> forward_flaw = get_flaw_legacy_forward(solution);
        if (forward_flaw) {
            return SplitProperties(get_split_from_flaw(*forward_flaw, false, split_unwanted_values),
                                   get_plan_perc(forward_flaw->abstract_state_id, solution), false, 1, 0);
        }
    }

    return SplitProperties(nullptr, backward, 0, 0);
}

SplitProperties FlawSearch::get_split_legacy_closest_to_goal(
    const Solution &solution,
    const bool split_unwanted_values) {
    if (log.is_at_least_debug()) {
        log << "Abstraction: " << endl;
        abstraction.dump();
    }

    unique_ptr<LegacyFlaw> forward_flaw = get_flaw_legacy_forward(solution);
    unique_ptr<LegacyFlaw> backward_flaw = get_flaw_legacy_backward(solution);
    bool backward_chosen = true;

    if (!backward_flaw && !forward_flaw) {
        return SplitProperties(nullptr, 0, false);
    } else if (!backward_flaw) {
        backward_chosen = false;
    } else if (shortest_paths.get_64bit_goal_distance(backward_flaw->abstract_state_id) >
               shortest_paths.get_64bit_goal_distance(forward_flaw->abstract_state_id)) {
        backward_chosen = false;
    }

    if (backward_chosen) {
        return SplitProperties(get_split_from_flaw(*backward_flaw, true, split_unwanted_values),
                               get_plan_perc(backward_flaw->abstract_state_id, solution), true);
    } else {
        return SplitProperties(get_split_from_flaw(*forward_flaw, false, split_unwanted_values),
                               get_plan_perc(forward_flaw->abstract_state_id, solution), false);
    }
}

unique_ptr<LegacyFlaw> FlawSearch::get_flaw_legacy_forward(const Solution &solution) {
    vector<LegacyFlaw> flaws = get_forward_flaws(solution, false, InAbstractionFlawSearchKind::FALSE);
    if (flaws.empty()) {
        return nullptr;
    } else {
        return utils::make_unique_ptr<LegacyFlaw>(move(flaws[0]));
    }
}

unique_ptr<LegacyFlaw> FlawSearch::get_flaw_legacy_backward(const Solution &solution) {
    vector<LegacyFlaw> flaws = get_backward_flaws(solution, false, InAbstractionFlawSearchKind::FALSE);
    if (flaws.empty()) {
        return nullptr;
    } else {
        return utils::make_unique_ptr<LegacyFlaw>(move(flaws[0]));
    }
}
}
