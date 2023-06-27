#include "symbolic_uniform_backwards_search_heuristic.h"

#include "../symbolic/original_state_space.h"
#include "../symbolic/searches/uniform_cost_search.h"

using namespace options;
using namespace std;
using namespace symbolic;

namespace cegar_symbolic_comparison {
SymUniformBackSearch::SymUniformBackSearch(const options::Options &opts,
                                           std::shared_ptr < symbolic::SymStateSpaceManager > originalStateSpace,
                                           std::shared_ptr < symbolic::SymVariables > vars)
    : SymbolicSearch(opts, vars, originalStateSpace->getParams()) {
    mgr = originalStateSpace;
    initialize();
}
void SymUniformBackSearch::initialize() {
    SymbolicSearch::initialize();
}

void SymUniformBackSearch::new_solution(const SymSolutionCut &sol) {
    if (!solution_registry->found_all_plans() && sol.get_f() < upper_bound) {
        solution_registry->register_solution(sol);
        upper_bound = sol.get_f();
    }
}
void SymUniformBackSearch::search(double generationTime, double generationMemory) {
    uc_search = make_shared<UniformCostSearch>(this, searchParams);
    uc_search->init(mgr, false, nullptr);
    plan_data_base->init(vars, task, plan_manager);
    auto individual_trs = uc_search->getStateSpaceShared()->getIndividualTRs();
    solution_registry->init(vars, nullptr, uc_search->getClosedShared(),
                            individual_trs,
                            plan_data_base, single_solution, simple);

    while (!uc_search->finished() &&
           (generationTime < 0.0001 || utils::g_timer() < generationTime) &&
           (generationMemory == 0 || (mgr->getVars()->totalMemory()) < generationMemory) &&
           !solved()) {
        if (!uc_search->step()) {
            break;
        }
    }

    uc_search->closeMinOpenAndCheckCut();
    cout << "Finished symbolic uniform backwards search" << endl;
}

ADD SymUniformBackSearch::getHeuristic() const {
    assert(uc_search);
    return uc_search->getClosedShared()->getHeuristic();
}





SymUniformBackSearchHeuristic::SymUniformBackSearchHeuristic(const Options &opts)
    : Heuristic(opts),
      task(tasks::g_root_task) {
}
SymUniformBackSearchHeuristic::SymUniformBackSearchHeuristic(const Options &opts,
                                                             shared_ptr < SymVariables > vars,
                                                             double max_time)
    : Heuristic(opts),
      vars(vars),
      max_time(max_time),
      task(tasks::g_root_task) {
    initialize(opts);
}
SymUniformBackSearchHeuristic::SymUniformBackSearchHeuristic(const Options &opts,
                                                             shared_ptr < SymVariables > vars,
                                                             double max_time,
                                                             const shared_ptr < AbstractTask > task)
    : Heuristic(opts),
      vars(vars),
      max_time(max_time),
      task(task) {
    initialize(opts);
}

void SymUniformBackSearchHeuristic::initialize_from_parser(const options::Options &opts) {
    vars = make_shared<SymVariables>(opts, task);
    vars->init();
    max_time = opts.get<double>("symbw_time");

    initialize(opts);
}

void SymUniformBackSearchHeuristic::initialize(const Options &opts) {
    SymParamsMgr mgrParams(opts, task);
    auto originalStateSpace = make_shared < OriginalStateSpace > (vars.get(), mgrParams, task);

    notMutexBDDs = originalStateSpace->getNotMutexBDDs(true);

    search_engine = unique_ptr < SymUniformBackSearch > (new SymUniformBackSearch(opts, originalStateSpace, vars));
    search_engine->search(max_time);

    heuristic = unique_ptr < ADD > (new ADD(std::move(search_engine->getHeuristic())));
}

int SymUniformBackSearchHeuristic::compute_heuristic(const State &state) {
    int *inputs = vars->getBinaryDescription(state);
    for (const BDD &bdd : notMutexBDDs) {
        if (bdd.Eval(inputs).IsZero()) {
            return DEAD_END;
        }
    }

    int res = 0;

    if (heuristic) {
        ADD evalNode = heuristic->Eval(inputs);
        int abs_cost = Cudd_V(evalNode.getRegularNode());

        if (abs_cost == -1)
            return DEAD_END;
        else if (abs_cost > res)
            res = abs_cost;
    }

    return res;
}
}
