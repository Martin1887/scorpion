#ifndef SYMBOLIC_GAMER_PDBS_HEURISTIC_H
#define SYMBOLIC_GAMER_PDBS_HEURISTIC_H

#include "../search_algorithms/symbolic_search.h"
#include "../../heuristic.h"


namespace symbolic {
class SymStateSpaceManager;
class OriginalStateSpace;
class UniformCostSearch;

class GamerPDBsHeuristic;

class PDBSearch : public SymbolicSearch {
    GamerPDBsHeuristic *spdbheuristic;
    std::set < int > pattern;
    std::shared_ptr < SymStateSpaceManager > state_space;
    std::unique_ptr < UniformCostSearch > uc_search;
    double average_hval;
    const std::shared_ptr < AbstractTask > task;

public:

    PDBSearch(GamerPDBsHeuristic *spdbheuristic,
              std::shared_ptr < SymStateSpaceManager > originalStateSpace,
              std::shared_ptr < SymVariables > vars,
              const plugins::Options &opts,
              const std::shared_ptr < AbstractTask > task = tasks::g_root_task);

    PDBSearch(const std::set < int > &pattern,
              GamerPDBsHeuristic *spdbheuristic,
              const std::shared_ptr < OriginalStateSpace > &originalStateSpace,
              std::shared_ptr < SymVariables > vars,
              const plugins::Options &opts,
              const std::shared_ptr < AbstractTask > task = tasks::g_root_task);


    void search(int generationTime = 0, double generationMemory = 0);

    ADD getHeuristic() const;
    double average_value();

    const std::set < int > &get_pattern() const {
        return pattern;
    }

    std::vector < int > candidate_vars() const;

    UniformCostSearch *get_search() {
        return uc_search.get();
    }
};

std::ostream &operator <<(std::ostream &os, const PDBSearch &pdb);

class GamerPDBsHeuristic : public Heuristic {
    const int generationTime;
    const double generationMemory;
    const bool useSuperPDB;
    const bool perimeter;

    int max_perimeter_heuristic;
    std::unique_ptr < ADD > perimeter_heuristic;
    std::unique_ptr < ADD > heuristic;
    std::vector < BDD > notMutexBDDs;

    std::shared_ptr < SymVariables > vars;

    const std::shared_ptr < AbstractTask > task;

    void dump_options() const;

    bool influences(int var, const std::set < int > &pattern);

    void initialize(const plugins::Options &opts);
protected:

    virtual int compute_heuristic(const State &ancestor_state) override;

public:
    GamerPDBsHeuristic(const plugins::Options &opts);
    GamerPDBsHeuristic(const plugins::Options &opts,
                       const std::shared_ptr < AbstractTask > task);
    virtual ~GamerPDBsHeuristic() = default;
};
}

#endif
