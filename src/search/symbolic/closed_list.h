#ifndef SYMBOLIC_CLOSED_LIST_H
#define SYMBOLIC_CLOSED_LIST_H

#include "searches/uniform_cost_search.h"
#include "sym_state_space_manager.h"
#include "sym_variables.h"

#include <map>
#include <set>
#include <vector>

namespace symbolic {
class SymSolutionCut;
class UniformCostSearch;
class SymSearch;

class ClosedList {
private:
    SymStateSpaceManager *mgr; // Symbolic manager to perform bdd operations

    std::map<int, BDD> closed; // Mapping from cost to set of states

    // Auxiliary BDDs for the number of 0-cost action steps
    // ALERT: The information here might be wrong
    // It is just used to extract path more quickly, but the information
    // here is an (admissible) estimation and this should be taken into account
    std::map<int, std::vector<BDD>> zeroCostClosed;
    BDD closedTotal; // All closed states.

    int hNotClosed, fNotClosed; // Bounds on h and g for those states not in closed
    std::map<int, BDD> closedUpTo;  // Disjunction of BDDs in closed  (auxiliary useful to take the maximum between several BDDs)
    std::set<int> h_values; //Set of h_values of the heuristic

    void newHValue(int h_value);

public:
    ClosedList();
    virtual ~ClosedList() {}
    void init(SymStateSpaceManager *manager);
    void init(SymStateSpaceManager *manager, const ClosedList &other);

    void insert(int h, const BDD &S);
    void setHNotClosed(int h);
    void setFNotClosed(int f);

    BDD getPartialClosed(int upper_bound) const;

    virtual SymSolutionCut getCheapestCut(const BDD &states, int g,
                                          bool fw) const;

    virtual std::vector<SymSolutionCut>
    getAllCuts(const BDD &states, int g, bool fw, int lower_bound) const;

    inline BDD getClosed() const {return closedTotal;}

    virtual BDD notClosed() const {return !closedTotal;}

    inline std::map<int, BDD> getClosedList() const {return closed;}

    inline int getHNotClosed() const {
        return hNotClosed;
    }

    inline int getFNotClosed() const {
        return fNotClosed;
    }

    void getHeuristic(std::vector<ADD> &heuristics,
                      std::vector <int> &maxHeuristicValues) const;

    ADD getHeuristic(int previousMaxH = -1) const;

    BDD get_start_states() const {
        if (get_num_zero_closed_layers(0) == 0) {
            return get_closed_at(0);
        }
        return get_zero_closed_at(0, 0);
    }

    inline BDD get_closed_at(int h) const {
        if (!closed.count(h)) {
            return mgr->zeroBDD();
        }
        return closed.at(h);
    }

    inline BDD get_zero_closed_at(int h, int layer) const {
        return zeroCostClosed.at(h).at(layer);
    }

    inline size_t get_num_zero_closed_layers(int h) const {
        if (zeroCostClosed.count(h) == 0) {
            return 0;
        }
        return zeroCostClosed.at(h).size();
    }

    inline size_t get_zero_cut(int h, const BDD &bdd) const {
        size_t i = 0;
        if (get_num_zero_closed_layers(h)) {
            for (; i < zeroCostClosed.at(h).size(); i++) {
                BDD intersection = zeroCostClosed.at(h).at(i) * bdd;
                if (!intersection.IsZero()) {
                    break;
                }
            }
        }
        return i;
    }

    double average_hvalue() const;
};
} // namespace symbolic

#endif
