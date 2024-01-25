#ifndef CARTESIAN_ABSTRACTIONS_ADDITIVE_CARTESIAN_HEURISTIC_H
#define CARTESIAN_ABSTRACTIONS_ADDITIVE_CARTESIAN_HEURISTIC_H

#include "../heuristic.h"

#include <vector>

namespace cartesian_abstractions {
class CartesianHeuristicFunction;

/*
  Store CartesianHeuristicFunctions and compute overall heuristic by
  summing all of their values.
*/
class AdditiveCartesianHeuristic : public Heuristic {
    std::vector<CartesianHeuristicFunction> heuristic_functions;

protected:
    virtual int compute_heuristic(const State &ancestor_state) override;
    virtual std::vector<CartesianHeuristicFunction> generate_heuristic_functions(
        const plugins::Options &opts, utils::LogProxy &log);

public:
    explicit AdditiveCartesianHeuristic(const plugins::Options &opts);
    void initialize(const plugins::Options &opts);
};
}

#endif
