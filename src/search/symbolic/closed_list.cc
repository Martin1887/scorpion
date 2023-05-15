#include "closed_list.h"

#include "plan_reconstruction/sym_solution_registry.h"
#include "sym_state_space_manager.h"
#include "sym_utils.h"

#include <cassert>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

using namespace std;

namespace symbolic {
ClosedList::ClosedList() : mgr(nullptr) {}

void ClosedList::init(SymStateSpaceManager *manager) {
    mgr = manager;
    set<int>().swap(h_values);
    map<int, BDD>().swap(closedUpTo);
    map<int, vector<BDD>>().swap(zeroCostClosed);
    map<int, BDD>().swap(closed);
    closedTotal = mgr->zeroBDD();
    hNotClosed = 0;
    fNotClosed = 0;
}

void ClosedList::init(SymStateSpaceManager *manager, const ClosedList &other) {
    mgr = manager;
    set<int>().swap(h_values);
    map<int, BDD>().swap(closedUpTo);
    map<int, vector<BDD>>().swap(zeroCostClosed);
    map<int, BDD>().swap(closed);
    closedTotal = mgr->zeroBDD();

    closedTotal = other.closedTotal;
    closed[0] = closedTotal;
}

void ClosedList::newHValue(int h_value) {
    h_values.insert(h_value);
}

void ClosedList::insert(int h, const BDD &S) {
    if (closed.count(h)) {
        assert(h_values.count(h));
        closed[h] += S;
    } else {
        closed[h] = S;
    }

    if (mgr->hasTransitions0()) {
        zeroCostClosed[h].push_back(S);
    }
    closedTotal += S;

    // Introduce in closedUpTo
    auto c = closedUpTo.lower_bound(h);
    while (c != std::end(closedUpTo)) {
        c->second += S;
        c++;
    }
}

void ClosedList::setHNotClosed(int newHNotClosed) {
    if (newHNotClosed > hNotClosed) {
        hNotClosed = newHNotClosed;
        newHValue(newHNotClosed);     //Add newHNotClosed to list of h values (and those of parents)
    }
}

void ClosedList::setFNotClosed(int f) {
    if (f > fNotClosed) {
        fNotClosed = f;
    }
}

BDD ClosedList::getPartialClosed(int upper_bound) const {
    BDD res = mgr->zeroBDD();
    for (const auto &pair : closed) {
        if (pair.first > upper_bound) {
            break;
        }
        res += pair.second;
    }
    return res;
}

SymSolutionCut ClosedList::getCheapestCut(const BDD &states, int g,
                                          bool fw) const {
    BDD cut_candidate = states * closedTotal;
    if (cut_candidate.IsZero()) {
        return SymSolutionCut();
    }

    for (const auto &closedH : closed) {
        int h = closedH.first;

        BDD cut = closedH.second * cut_candidate;
        if (!cut.IsZero()) {
            if (fw) {
                return SymSolutionCut(g, h, cut);
            } else {
                return SymSolutionCut(h, g, cut);
            }
        }
    }
    std::cerr << "Inconsistent cut result" << std::endl;
    exit(0);
    return SymSolutionCut();
}

std::vector<SymSolutionCut> ClosedList::getAllCuts(const BDD &states, int g,
                                                   bool fw,
                                                   int lower_bound) const {
    std::vector<SymSolutionCut> result;
    BDD cut_candidate = states * closedTotal;
    if (!cut_candidate.IsZero()) {
        for (const auto &closedH : closed) {
            int h = closedH.first;

            /* Here we also need to consider higher costs due to the architecture
             of symBD. Otherwise their occur problems in
             */
            if (g + h < lower_bound) {
                continue;
            }

            // cout << "Check cut of g=" << g << " with h=" << h << endl;
            BDD cut = closedH.second * cut_candidate;
            if (!cut.IsZero()) {
                if (fw) {
                    result.emplace_back(g, h, cut);
                } else {
                    result.emplace_back(h, g, cut);
                }
            }
        }
    }
    return result;
}

void ClosedList::getHeuristic(vector<ADD> &heuristics,
                              vector <int> &maxHeuristicValues) const {
    int previousMaxH = 0;     //Get the previous value of max h
    if (!maxHeuristicValues.empty()) {
        previousMaxH = maxHeuristicValues.back();
    }
    /*If we did not complete one step, and we do not surpass the previous maxH
      we do not have heuristic*/
    if (closed.size() <= 1 && hNotClosed <= previousMaxH) {
        cout << "Heuristic not inserted: "
             << hNotClosed << " " << closed.size() << endl;
        return;
    }

    ADD h = getHeuristic(previousMaxH);

    //  closed.clear(); //The heuristic values have been stored in the ADD.
    cout << "Heuristic with maxValue: " << hNotClosed
         << " ADD size: " << h.nodeCount() << endl;

    maxHeuristicValues.push_back(hNotClosed);
    heuristics.push_back(h);
}


ADD ClosedList::getHeuristic(int previousMaxH /*= -1*/) const {
    /* When zero cost operators have been expanded, all the states in non reached
       have a h-value strictly greater than frontierCost.
       They can be frontierCost + min_action_cost or the least bucket in open. */
    /*  int valueNonReached = frontierCost;
        if(frontierCost >= 0 && zeroCostExpanded){
        cout << "Frontier cost is " << frontierCost << endl;
        closed[frontierCost] = S;
        valueNonReached = frontierCost + mgr->getMinTransitionCost();
        if(!open.empty()){
        valueNonReached = min(open.begin()->first,
        valueNonReached);
        }
        }*/
    BDD statesWithHNotClosed = !closedTotal;
    ADD h = mgr->mgr()->constant(-1);
    // cout << "New heuristic with h [";
    for (auto &it : closed) {
        // cout << it.first << " ";
        int h_val = it.first;

        /*If h_val < previousMaxH we can put it to that value
          However, we only do so if it is less than hNotClosed
          (or we will think that we have not fully determined the value)*/
        if (h_val < previousMaxH && previousMaxH < hNotClosed) {
            h_val = previousMaxH;
        }
        if (h_val != hNotClosed) {
            h += it.second.Add() * mgr->mgr()->constant(h_val + 1);
        } else {
            statesWithHNotClosed += it.second;
        }
    }
    // cout << hNotClosed << "]" << endl;

    if (hNotClosed != numeric_limits<int>::max() && hNotClosed >= 0 && !statesWithHNotClosed.IsZero()) {
        h += statesWithHNotClosed.Add() * mgr->mgr()->constant(hNotClosed + 1);
    }

    return h;
}

double ClosedList::average_hvalue() const {
    double averageHeuristic = 0;
    double heuristicSize = 0;
    for (const auto &item : closed) {
        double currentSize = mgr->getVars()->numStates(item.second);
        averageHeuristic += currentSize * item.first;
        heuristicSize += currentSize;
    }
    double notClosedSize = mgr->getVars()->numStates(notClosed());
    heuristicSize += notClosedSize;
    int maxH = (closed.empty() ? 0 : closed.rbegin()->first);

    averageHeuristic += notClosedSize * maxH;
    return averageHeuristic / heuristicSize;
}
} // namespace symbolic
