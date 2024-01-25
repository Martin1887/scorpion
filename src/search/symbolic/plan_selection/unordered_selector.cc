#include "unordered_selector.h"

#include "../../plugins/options.h"
#include "../../state_registry.h"

using namespace std;

namespace symbolic {
UnorderedSelector::UnorderedSelector(const plugins::Options &opts) : PlanSelector(opts) {
    PlanSelector::anytime_completness = true;
}

void UnorderedSelector::add_plan(const Plan &plan) {
    Plan unordered = plan;
    sort(unordered.begin(), unordered.end());

    if (!has_accepted_plan(unordered)) {
        save_accepted_plan(plan, unordered);
    }
}

void UnorderedSelector::save_accepted_plan(const Plan &ordered_plan, const Plan &unordered_plan) {
    if (num_accepted_plans == 0) {
        first_accepted_plan = ordered_plan;
        first_accepted_plan_cost = calculate_plan_cost(
            ordered_plan, state_registry->get_task_proxy());
    }

    size_t plan_seed = get_hash_value(unordered_plan);
    if (hashes_accepted_plans.count(plan_seed) == 0) {
        hashes_accepted_plans[plan_seed] = vector<Plan>();
    }
    hashes_accepted_plans[plan_seed].push_back(unordered_plan);
    states_accepted_goal_paths += states_on_path(ordered_plan);
    num_accepted_plans++;
    plan_mgr.save_plan(ordered_plan, state_registry->get_task_proxy(),
                       false, true);
}

class UnorderedSelectorFeature : public plugins::TypedFeature<PlanSelector, UnorderedSelector> {
public:
    UnorderedSelectorFeature() : TypedFeature("unordered") {
        PlanSelector::add_options_to_feature(*this);
    }
};

static plugins::FeaturePlugin<UnorderedSelectorFeature> _plugin;
}
