#include "top_k_even_selector.h"

#include "../../plugins/options.h"

using namespace std;

namespace symbolic {
TopKEvenSelector::TopKEvenSelector(const plugins::Options &opts)
    : PlanSelector(opts) {
    anytime_completness = true;
}

void TopKEvenSelector::add_plan(const Plan &plan) {
    if (!has_rejected_plan(plan) && !has_accepted_plan(plan)) {
        if (plan.size() % 2 == 0) {
            save_accepted_plan(plan);
        } else {
            save_rejected_plan(plan);
        }
    }
}

class TopkEvenSelectorFeature : public plugins::TypedFeature<PlanSelector, TopKEvenSelector> {
public:
    TopkEvenSelectorFeature() : TypedFeature("top_k_even") {
        PlanSelector::add_options_to_feature(*this);
    }
};

static plugins::FeaturePlugin<TopkEvenSelectorFeature> _plugin;
}
