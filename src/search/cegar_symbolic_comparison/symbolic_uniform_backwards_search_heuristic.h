#ifndef SYMBOLIC_UNIFORM_BACKWARDS_SEARCH_HEURISTIC_H
#define SYMBOLIC_UNIFORM_BACKWARDS_SEARCH_HEURISTIC_H

#include "../symbolic/plan_reconstruction/sym_solution_cut.h"
#include "../symbolic/search_engines/symbolic_search.h"
#include "../symbolic/searches/uniform_cost_search.h"
#include "../heuristic.h"

namespace cegar_symbolic_comparison {
class SymUniformBackSearch : public symbolic::SymbolicSearch {
protected:
    std::shared_ptr < symbolic::UniformCostSearch > uc_search;

    virtual void initialize() override;

    virtual SearchStatus step() override {return SymbolicSearch::step();}
public:
    SymUniformBackSearch(const options::Options &opts,
                         std::shared_ptr < symbolic::SymStateSpaceManager > originalStateSpace,
                         std::shared_ptr < symbolic::SymVariables > vars);
    virtual ~SymUniformBackSearch() = default;

    virtual void new_solution(const symbolic::SymSolutionCut &sol) override;

    void search(int generationTime = 0, double generationMemory = 0);

    ADD getHeuristic() const;
};


class SymUniformBackSearchHeuristic : public Heuristic {
    std::unique_ptr < SymUniformBackSearch > search_engine;
    std::shared_ptr < symbolic::SymVariables > vars;
    const std::shared_ptr < AbstractTask > task;
    std::vector < BDD > notMutexBDDs;
    std::unique_ptr < ADD > heuristic;

    void initialize(const options::Options &opts);
protected:
    virtual int compute_heuristic(const State &ancestor_state) override;

public:
    SymUniformBackSearchHeuristic(const options::Options &opts,
                                  std::shared_ptr < symbolic::SymVariables > vars);
    SymUniformBackSearchHeuristic(const options::Options &opts,
                                  std::shared_ptr < symbolic::SymVariables > vars,
                                  const std::shared_ptr < AbstractTask > task);
    virtual ~SymUniformBackSearchHeuristic() = default;

    int h_value(const State &ancestor_state) {return compute_heuristic(ancestor_state);}
};
}

#endif
