#ifndef CEGAR_SYMBOLIC_COMPARISON_H
#define CEGAR_SYMBOLIC_COMPARISON_H

#include "../cartesian_abstractions/additive_cartesian_heuristic.h"

#include <vector>

namespace cegar_symbolic_comparison {
/*
  Store CartesianHeuristicFunctions and compute overall heuristic by
  summing all of their values.

  Only CEGAR heuristics are used for the search, the symbolic ones are
  only compared with them.
*/
class CegarSymbolicComparison : public cartesian_abstractions::AdditiveCartesianHeuristic {
protected:
    virtual std::vector < cartesian_abstractions::CartesianHeuristicFunction > generate_heuristic_functions(
        const plugins::Options &opts, utils::LogProxy &log) override;
public:
    explicit CegarSymbolicComparison(const plugins::Options &opts);
};
}

#endif
