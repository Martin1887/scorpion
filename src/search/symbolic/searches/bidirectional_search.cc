#include "bidirectional_search.h"

#include "../search_algorithms/symbolic_search.h"
#include <memory>

using namespace std;

namespace symbolic {
BidirectionalSearch::BidirectionalSearch(SymbolicSearch *eng,
                                         const SymParamsSearch &params,
                                         shared_ptr < UniformCostSearch > _fw,
                                         shared_ptr < UniformCostSearch > _bw)
    : SymSearch(eng, params), fw(_fw), bw(_bw), cur_dir(nullptr) {
    assert(fw->getStateSpace() == bw->getStateSpace());
    mgr = fw->getStateSpaceShared();
}

string BidirectionalSearch::get_last_dir() const {
    return cur_dir ? cur_dir->get_last_dir() : "";
}

UniformCostSearch *BidirectionalSearch::selectBestDirection() {
    Estimation &fw_est = *fw->get_step_estimator();
    Estimation &bw_est = *bw->get_step_estimator();
    if (fw_est.get_failed() && bw_est.get_failed()) {
        p.increase_bound();
        bw_est.set_data(bw_est.get_time(), bw_est.get_nodes(), false);
        return fw.get();
    }
    cur_dir = (bw_est < fw_est) ? bw : fw;
    return cur_dir.get();
}

bool BidirectionalSearch::finished() const {
    return fw->finished() || bw->finished();
}

bool BidirectionalSearch::stepImage(int maxTime, int maxNodes) {
    bool res = selectBestDirection()->stepImage(maxTime, maxNodes);
    engine->setLowerBound(getF());
    engine->setMinG(fw->getG() + bw->getG());


    if (engine->solved()) {
        fw->closeMinOpenAndCheckCut();
        bw->closeMinOpenAndCheckCut();
    }

    return res;
}
} // namespace symbolic
