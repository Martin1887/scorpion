#ifndef CEGAR_SYMBOLIC_COMPARISON_H
#define CEGAR_SYMBOLIC_COMPARISON_H

#include "../cegar/additive_cartesian_heuristic.h"

#include <vector>

namespace cegar_symbolic_comparison {
/*
  Store CartesianHeuristicFunctions and compute overall heuristic by
  summing all of their values.

  Only CEGAR heuristics are used for the search, the symbolic ones are
  only compared with them.
*/
    class CegarSymbolicComparison: public cegar::AdditiveCartesianHeuristic {
protected:
        virtual std::vector < cegar::CartesianHeuristicFunction > generate_heuristic_functions(
            const options::Options &opts, utils::LogProxy &log) override;
public:
        explicit CegarSymbolicComparison(const options::Options &opts);
    };
}

#endif
