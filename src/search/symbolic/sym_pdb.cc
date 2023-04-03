#include "sym_pdb.h"

#include "original_state_space.h"
#include "sym_utils.h"
#include "transition_relation.h"

#include "../utils/system.h"

using namespace std;

namespace symbolic {
SymPDB::SymPDB(const OriginalStateSpace &parent,
               const std::set < int > &relevantVars)
    : SymStateSpaceManager(parent.getVars(), parent.getParams(), relevantVars) {
    std::set < int > nonRelVars;
    for (int i = 0; i < tasks::g_root_task->get_num_variables(); ++i) {
        if (!isRelevantVar(i)) {
            nonRelVars.insert(i);
        }
    }

    nonRelVarsCube =
        vars->getCubePre(nonRelVars);     // * vars->getCubep(nonRelVars);
    assert(nonRelVarsCube.IsCube());
    nonRelVarsCubeWithPrimes = nonRelVarsCube * vars->getCubeEff(nonRelVars);

    // Init initial state
    vector < pair < int, int >> abstract_ini;
    vector < int > initial_values = tasks::g_root_task->get_initial_state_values();
    for (int var : relevant_vars) {
        abstract_ini.push_back(std::pair < int, int > (var, initial_values[var]));
    }
    initialState = vars->getPartialStateBDD(abstract_ini);

    // Init goal
    vector < pair < int, int >> abstract_goal;
    for (int goal_index = 0; goal_index < tasks::g_root_task->get_num_goals(); goal_index++) {
        FactPair goal_var = tasks::g_root_task->get_goal_fact(goal_index);
        if (isRelevantVar(goal_var.var)) {
            abstract_goal.push_back(std::pair < int, int > (goal_var.var, goal_var.value));
        }
    }
    goal = vars->getPartialStateBDD(abstract_goal);

    // Init mutex

    // Init dead ends: Both are put into notDeadEndFw for the case
    // of abstract searches
    for (const auto &bdd : parent.getNotDeadEnds(false)) {
        notDeadEndFw.push_back(bdd);
    }

    for (const auto &bdd : parent.getNotDeadEnds(true)) {
        notDeadEndFw.push_back(bdd);
    }

    mergeBucketAnd(notDeadEndFw);

    for (auto &bdd : notDeadEndFw) {
        bdd = shrinkExists(bdd, p.max_mutex_size);
    }

    // Init transitions
    std::map < int, std::vector < TransitionRelation >> indTRs;
    std::map < int, std::vector < TransitionRelation >> failedToShrink;
    for (const auto &indTRsCost : parent.getIndividualTRs()) {
        for (const auto &trParent : indTRsCost.second) {
            TransitionRelation absTransition = TransitionRelation(trParent);
            assert(absTransition.getCost() == indTRsCost.first);
            assert(absTransition.getOpsIds().size() == 1);
            if (!is_relevant_op(*(absTransition.getOpsIds().begin())))
                continue;

            int cost = absTransition.getCost();
            try {
                vars->setTimeLimit(p.max_aux_time);
                absTransition.shrink(*this, p.max_aux_nodes);
                vars->unsetTimeLimit();
                indTRs[cost].push_back(absTransition);
            }catch (BDDError e) {
                vars->unsetTimeLimit();
                failedToShrink[cost].push_back(absTransition);
            }
        }
    }

    init_transitions(indTRs);

    for (auto &trs : transitions) {
        merge(vars, trs.second, mergeTR, p.max_aux_time, p.max_tr_size);
    }

    // Use Shrink after img in all the transitions that failedToShrink
    for (auto &failedTRs : failedToShrink) {
        merge(vars, failedTRs.second, mergeTR, p.max_aux_time, p.max_tr_size);
        for (auto &tr : failedTRs.second) {
            tr.setAbsAfterImage(this);
            transitions[failedTRs.first].push_back(tr);
        }
    }

    assert(!hasTR0 || transitions.count(0));
}

BDD SymPDB::shrinkExists(const BDD &bdd, int maxNodes) const {
    return bdd.ExistAbstract(nonRelVarsCube, maxNodes);
}

BDD SymPDB::shrinkTBDD(const BDD &bdd, int maxNodes) const {
    return bdd.ExistAbstract(nonRelVarsCubeWithPrimes, maxNodes);
}

BDD SymPDB::shrinkForall(const BDD &bdd, int maxNodes) const {
    return bdd.UnivAbstract(nonRelVarsCube, maxNodes);
}

// void SymPDB::init_mutex(const std::vector<MutexGroup> & mutex_groups) {
//      // if(/*p.init_mutex_from_parent &&*/ parentMgr){
//      //     setTimeLimit(p.max_mutex_time);
//      //     DEBUG_MSG(cout << "Init mutex from parent" << endl;);
//      //     mutexInitialized = true;
//      //     //Initialize mutexes from other manager
//      //     try{
//      //      for(auto & bdd : parentMgr->notMutexBDDsFw){
//      //          BDD shrinked = abstraction->shrinkExists(bdd,
//      p.max_mutex_size);
//      //          notMutexBDDsFw.push_back(shrinked);
//      //      }
//      //      for(auto & bdd : parentMgr->notMutexBDDsBw){
//      //          BDD shrinked = abstraction->shrinkExists(bdd,
//      p.max_mutex_size);
//      //          notMutexBDDsBw.push_back(shrinked);
//      //      }
//      //      unsetTimeLimit();
//      //     }catch(BDDError e){
//      //      unsetTimeLimit();
//      //      //Forget about it
//      //      vector<BDD>().swap(notMutexBDDsFw);
//      //      vector<BDD>().swap(notMutexBDDsBw);
//      //      init_mutex(mutex_groups, true, false);
//      //     }
// }

std::string SymPDB::tag() const {return "PDB";}

void SymPDB::print(std::ostream &os, bool fullInfo) const {
    os << "PDB (" << relevant_vars.size() << "/" << (tasks::g_root_task->get_num_variables())
       << "): ";
    for (int v : relevant_vars) {
        os << v << " ";
    }
    if (fullInfo) {
        os << " [";
        for (int v : relevant_vars)
            os << v << " ";
        os << "]";
        os << endl << "Considered propositions: ";
        for (int v : relevant_vars) {
            os << v << ": ";
            for (int i = 0; i < tasks::g_root_task->get_variable_domain_size(v); i++) {
                os << tasks::g_root_task->get_fact_name(FactPair(v, i)) << ", ";
            }
            os << endl;
        }
        os << endl;
    }
}
} // namespace symbolic
