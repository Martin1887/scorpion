#include "uniform_cost_search.h"

#include "../closed_list.h"
#include "../frontier.h"
#include "../plan_reconstruction/sym_solution_cut.h"
#include "../search_engines/symbolic_search.h"
#include "../sym_utils.h"
#include "../utils/timer.h"

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

using namespace std;

namespace symbolic {
UniformCostSearch::UniformCostSearch(SymbolicSearch *eng,
                                     const SymParamsSearch &params)
    : SymSearch(eng, params),
      fw(true),
      step_estimation(0, 0, false),
      closed(make_shared < ClosedList > ()),
      lastStepCost(true) {}

bool UniformCostSearch::init(shared_ptr < SymStateSpaceManager > manager,
                             bool forward, UniformCostSearch *opposite_search) {
    mgr = manager;
    fw = forward;
    lastStepCost = true;
    last_g_cost = 0;
    assert(mgr);

    BDD init_bdd = fw ? mgr->getInitialState() : mgr->getGoal();
    frontier.init(manager.get(), init_bdd);

    closed->init(mgr.get());
    closed->insert(0, init_bdd);
    closed->setHNotClosed(open_list.minNextG(frontier, mgr->getAbsoluteMinTransitionCost()));
    closed->setFNotClosed(getF());

    if (opposite_search) {
        perfectHeuristic = opposite_search->getClosedShared();
    } else {
        perfectHeuristic = make_shared < ClosedList > ();
        perfectHeuristic->init(mgr.get());
        if (fw) {
            perfectHeuristic->insert(0, mgr->getGoal());
        } else {
            perfectHeuristic->insert(0, mgr->getInitialState());
        }
    }

    prepareBucket();

    engine->setLowerBound(getF());
    engine->setMinG(getG());

    return true;
}

void UniformCostSearch::checkFrontierCut(Bucket &bucket, int g) {
    if (p.get_non_stop()) {
        return;
    }

    for (BDD &bucketBDD : bucket) {
        auto sol = perfectHeuristic->getCheapestCut(bucketBDD, g, fw);
        if (sol.get_f() >= 0) {
            engine->new_solution(sol);
        }
        // Prune everything closed in opposite direction
        bucketBDD *= perfectHeuristic->notClosed();
    }
}


void UniformCostSearch::closeMinOpenAndCheckCut() {
    int up_to = frontier.g() + mgr->getAbsoluteMinTransitionCost();
    while (!open_list.empty() && frontier.g() < up_to) {
        prepareBucket();
        frontier.make_empty();
    }
}


bool UniformCostSearch::provable_no_more_plans() {return open_list.empty();}

bool UniformCostSearch::prepareBucket() {
    if (!frontier.bucketReady()) {
        if (open_list.empty()) {
            closed->setHNotClosed(numeric_limits<int>::max());
            closed->setFNotClosed(numeric_limits<int>::max());
        }
        if (provable_no_more_plans()) {
            engine->setLowerBound(numeric_limits < int > ::max());
            return true;
        }

        open_list.pop(frontier);
        last_g_cost = frontier.g();
        assert(!frontier.empty() || frontier.g() == numeric_limits < int > ::max());
        checkFrontierCut(frontier.bucket(), frontier.g());

        filterFrontier();

        // Close and move to reopen

        if (!lastStepCost || frontier.g() != 0) {
            // Avoid closing init twice
            for (const BDD &states : frontier.bucket()) {
                closed->insert(frontier.g(), states);
            }
        }
        engine->setLowerBound(getF());
        engine->setMinG(getG());
        closed->setHNotClosed(open_list.minNextG(frontier, mgr->getAbsoluteMinTransitionCost()));
        closed->setFNotClosed(getF());
    }

    if (engine->solved()) {
        return true;     // If it has been solved, return
    }

    return false;
}

// Here we filter states: remove closed states and mutex states
// This procedure is delayed in comparison to explicit search
// Idea: no need to "change" BDDs until we actually process them
void UniformCostSearch::filterFrontier() {
    frontier.filter(!closed->notClosed());
    mgr->filterMutex(frontier.bucket(), fw, initialization());
    removeZero(frontier.bucket());
}

bool UniformCostSearch::stepImage(int maxTime, int maxNodes) {
    if (p.debug) {
        cout << ">> Step: " << *mgr << (fw ? " fw " : " bw ") << ", g=" << frontier.g()
             << " frontierNodes: " << frontier.nodes() << " [" << frontier.buckets() << "]"
             << " total time: " << utils::g_timer << endl;
    }
    utils::Timer step_timer;

    bool done = prepareBucket();

    if (done) {
        return true;
    }

    Result prepare_res =
        frontier.prepare(maxTime, maxNodes, fw, initialization());
    if (!prepare_res.ok) {
        step_estimation.set_data(step_timer(), frontier.nodes(), !prepare_res.ok);
        return false;
    }

    if (engine->solved()) {
        return true;     // Skip image if we are done
    }

    int stepNodes = frontier.nodes();
    ResultExpansion res_expansion = frontier.expand(maxTime, maxNodes, fw);

    if (res_expansion.ok) {
        lastStepCost = false;     // Must be set to false before calling checkCut
        // Process Simg, removing duplicates and computing h. Store in Sfilter and
        // reopen. Include new states in the open list
        for (auto &resImage : res_expansion.buckets) {
            for (auto &pairCostBDDs : resImage) {
                int cost = frontier.g() +
                    pairCostBDDs.first;     // Include states of the given cost
                mgr->mergeBucket(pairCostBDDs.second);

                // Check for cut (remove those states)
                checkFrontierCut(pairCostBDDs.second, cost);

                for (auto &bdd : pairCostBDDs.second) {
                    if (!bdd.IsZero()) {
                        stepNodes = max(stepNodes, bdd.nodeCount());
                        open_list.insert(bdd, cost);
                    }
                }
            }
        }
    }

    // We prepare the next bucket before checking time in doing
    // the step because we consider preparing the bucket as a
    // part of the step.
    prepareBucket();

    engine->setLowerBound(getG() + mgr->getAbsoluteMinTransitionCost());

    step_estimation.set_data(step_timer(), stepNodes, !res_expansion.ok);

    return res_expansion.ok;
}

bool UniformCostSearch::isSearchableWithNodes(int maxNodes) const {
    return frontier.expansionReady() && nextStepNodes() <= maxNodes;
}
} // namespace symbolic
