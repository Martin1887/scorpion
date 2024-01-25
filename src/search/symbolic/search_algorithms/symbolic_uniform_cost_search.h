#ifndef SYMBOLIC_SEARCH_ENGINES_SYMBOLIC_UNIFORM_COST_SEARCH_H
#define SYMBOLIC_SEARCH_ENGINES_SYMBOLIC_UNIFORM_COST_SEARCH_H

#include "symbolic_search.h"

namespace symbolic {
class SymbolicUniformCostSearch : public SymbolicSearch {
protected:
    bool fw;
    bool bw;

    virtual void initialize() override;

    virtual SearchStatus step() override {return SymbolicSearch::step();}

public:
    SymbolicUniformCostSearch(const plugins::Options &opts,
                              const std::shared_ptr < AbstractTask > task = tasks::g_root_task);
    virtual ~SymbolicUniformCostSearch() = default;

    virtual void new_solution(const SymSolutionCut &sol) override;
};
} // namespace symbolic

#endif
