#include "top_q_symbolic_uniform_cost_search.h"

#include "../original_state_space.h"
#include "../plugins/plugin.h"
#include "../searches/bidirectional_search.h"
#include "../searches/top_k_uniform_cost_search.h"
#include "../../plugins/options.h"

using namespace std;

namespace symbolic {
TopqSymbolicUniformCostSearch::TopqSymbolicUniformCostSearch(
    const plugins::Options &opts)
    : TopkSymbolicUniformCostSearch(opts),
      quality_multiplier(opts.get<double>("quality")) {
    utils::g_log << "Quality: " << quality_multiplier << endl;
}

void TopqSymbolicUniformCostSearch::new_solution(const SymSolutionCut &sol) {
    if (!(solution_registry->found_all_plans() ||
          lower_bound > get_quality_bound())) {
        solution_registry->register_solution(sol);
        if (get_quality_bound() < numeric_limits<double>::infinity()) {
            // utils::g_log << "Quality bound: " << get_quality_bound() << endl;
            upper_bound = min(upper_bound, (int)get_quality_bound() + 1);
        }
    } else {
        lower_bound = numeric_limits<int>::max();
    }
}

SearchStatus TopqSymbolicUniformCostSearch::step() {
    step_num++;
    // Handling empty plan
    if (step_num == 0) {
        BDD cut = mgr->getInitialState() * mgr->getGoal();
        if (!cut.IsZero()) {
            new_solution(SymSolutionCut(0, 0, cut));
        }
    }

    SearchStatus cur_status;

    // Search finished!
    if (lower_bound >= upper_bound) {
        solution_registry->construct_cheaper_solutions(upper_bound);
        solution_found = plan_data_base->get_num_reported_plan() > 0;
        cur_status = solution_found ? SOLVED : FAILED;
    } else {
        // Bound increade => construct plans
        if (lower_bound_increased) {
            solution_registry->construct_cheaper_solutions(lower_bound);
        }

        // All plans found
        if (solution_registry->found_all_plans()) {
            solution_found = true;
            cur_status = SOLVED;
        } else {
            cur_status = IN_PROGRESS;
        }
    }

    if (lower_bound_increased && !silent) {
        utils::g_log << "BOUND: " << lower_bound << " < " << upper_bound << flush;

        utils::g_log << " [" << solution_registry->get_num_found_plans() << "/"
                     << plan_data_base->get_num_desired_plans() << " plans]"
                     << flush;
        utils::g_log << ", total time: " << utils::g_timer << endl;
    }
    lower_bound_increased = false;

    if (cur_status == SOLVED) {
        set_plan(plan_data_base->get_first_accepted_plan());
        cout << endl;
        return cur_status;
    }
    if (cur_status == FAILED) {
        return cur_status;
    }

    // Actuall step
    search->step();

    return cur_status;
}

void TopqSymbolicUniformCostSearch::add_options_to_feature(plugins::Feature &feature) {
    feature.add_option<double>("quality", "relative quality multiplier",
                               "infinity", plugins::Bounds("1.0", "infinity"));
}
} // namespace symbolic
class TopqSymbolicUniformCostSearchFeature : public plugins::TypedFeature<SearchAlgorithm, symbolic::TopqSymbolicUniformCostSearch> {
public:
    TopqSymbolicUniformCostSearchFeature() : TypedFeature("symq") {
        document_synopsis("Top-q Symbolic Bidirectional Uniform Cost Search");
        symbolic::SymbolicSearch::add_options_to_feature(*this);
        add_option<shared_ptr<symbolic::PlanSelector>>(
            "plan_selection", "plan selection strategy");
        symbolic::TopqSymbolicUniformCostSearch::add_options_to_feature(*this);
        add_option<bool>("fw", "Search in the forward direction", "false");
        add_option<bool>("bw", "Search in the backward direction", "false");
    }
};

static plugins::FeaturePlugin<TopqSymbolicUniformCostSearchFeature> _plugin;
