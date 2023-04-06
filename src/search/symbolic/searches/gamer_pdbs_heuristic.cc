#include "gamer_pdbs_heuristic.h"

#include "uniform_cost_search.h"

#include "../option_parser.h"
#include "../original_state_space.h"
#include "../plugin.h"
#include "../sym_enums.h"
#include "../sym_pdb.h"
#include "../sym_variables.h"
#include "../../task_utils/causal_graph.h"

#include <cassert>
#include <limits>
#include <memory>
#include <vector>

using namespace std;

namespace symbolic {
PDBSearch::PDBSearch (GamerPDBsHeuristic *spdbheuristic_,
                      shared_ptr < SymStateSpaceManager > originalStateSpace,
                      shared_ptr < SymVariables > vars,
                      const options::Options &opts,
                      const std::shared_ptr < AbstractTask > task) :
    SymbolicSearch(opts, vars, originalStateSpace->getParams()),
    spdbheuristic(spdbheuristic_), state_space(originalStateSpace),
    average_hval(-1), task(task) {
    initialize();
    for (int i = 0; i < task->get_num_variables(); ++i) {
        pattern.insert(i);
    }
}


PDBSearch::PDBSearch (const set < int > &pattern_,
                      GamerPDBsHeuristic *spdbheuristic_,
                      const shared_ptr < OriginalStateSpace > &originalStateSpace,
                      shared_ptr < SymVariables > vars,
                      const options::Options &opts,
                      const std::shared_ptr < AbstractTask > task) :
    SymbolicSearch(opts, vars, originalStateSpace->getParams()),
    spdbheuristic(spdbheuristic_), pattern(pattern_),
    average_hval(-1), task(task) {
    initialize();
    if ((int)pattern.size() != task->get_num_variables()) {
        state_space = make_shared < SymPDB > (*originalStateSpace, task, pattern_);
    } else {
        state_space = originalStateSpace;
    }
}

std::ostream &operator <<(std::ostream &os, const PDBSearch &pdb) {
    for (int v : pdb.get_pattern())
        os << " " << v;
    return os;
}


void PDBSearch::search(int generationTime, double generationMemory) {
    uc_search = unique_ptr < UniformCostSearch > (new UniformCostSearch(this, searchParams));
    uc_search->init(state_space, false, nullptr);

    while (!uc_search->finished() &&
           (generationTime == 0 || utils::g_timer() < generationTime) &&
           (generationMemory == 0 || (state_space->getVars()->totalMemory()) < generationMemory) &&
           !solved()) {
        if (!uc_search->step())
            break;
    }

    assert(!uc_search->finished() ||
           solved() ||
           state_space->isAbstracted());
    average_hval = uc_search->getClosedShared()->average_hvalue();
    cout << "Finished PDB: " << *this << flush << "   Average value: " << average_hval << " g_time: " << utils::g_timer() << endl;
}


vector < int > PDBSearch::candidate_vars() const {
    TaskProxy task_proxy(*task);
    const causal_graph::CausalGraph &cg = task_proxy.get_causal_graph();
    vector < int > candidates;
    for (size_t var = 0; var < task_proxy.get_variables().size(); ++var) {
        if (pattern.count(var))
            continue;

        for (int succ : cg.get_pre_to_eff(var)) {
            if (pattern.count(succ)) {
                candidates.push_back(var);
                break;
            }
        }
    }

    return candidates;
}


double PDBSearch::average_value() {
    if (!uc_search)
        return 0;
    if (average_hval == -1) {
        utils::Timer t;
        average_hval = uc_search->getClosedShared()->average_hvalue();
        cout << "Time average: " << t() << endl;
    }

    return average_hval;
}


ADD PDBSearch::getHeuristic() const {
    assert(uc_search);
    return uc_search->getClosedShared()->getHeuristic();
}


GamerPDBsHeuristic::GamerPDBsHeuristic(const Options &opts)
    : Heuristic(opts),
      generationTime(opts.get < int > ("generation_time")),
      generationMemory(opts.get < double > ("generation_memory")),
      useSuperPDB(opts.get < bool > ("super_pdb")),
      perimeter(opts.get < bool > ("perimeter")),
      task(tasks::g_root_task) {
    initialize(opts);
}
GamerPDBsHeuristic::GamerPDBsHeuristic(const Options &opts,
                                       const std::shared_ptr < AbstractTask > task)
    : Heuristic(opts, task),
      generationTime(opts.get < int > ("generation_time")),
      generationMemory(opts.get < double > ("generation_memory")),
      useSuperPDB(opts.get < bool > ("super_pdb")),
      perimeter(opts.get < bool > ("perimeter")),
      task(task) {
    initialize(opts);
}


void GamerPDBsHeuristic::initialize(const Options &opts) {
    utils::Timer timer;
    cout << "Initializing gamer pdb heuristic..." << endl;
    dump_options();

    // Get mutex fw BDDs to detect spurious states as dead ends

    cout << "Initialize original search" << endl;

    SymParamsMgr mgrParams(opts);
    cout << "mgrParams created";
    vars = make_shared < SymVariables > (opts, task);
    vars->init();
    auto originalStateSpace = make_shared < OriginalStateSpace > (vars.get(), mgrParams, task);


    notMutexBDDs = originalStateSpace->getNotMutexBDDs(true);

    cout << "Use super: " << useSuperPDB << endl;
    cout << "Use perimeter: " << perimeter << endl;

    if (useSuperPDB || perimeter) {
        PDBSearch pdb_search(this, originalStateSpace, vars, opts, task);

        pdb_search.search(generationTime, generationMemory);
        cout << "Finished super PDB: " << endl;

        if (pdb_search.found_solution()) {
            cout << "Problem solved during heuristic generation" << endl;
            // heuristic = make_unique<ADD>(solution.getADD());
            // return;
        }

        if (perimeter) {
            UniformCostSearch *search = pdb_search.get_search();
            perimeter_heuristic = unique_ptr < ADD > (new ADD(std::move(search->getClosedShared()->getHeuristic())));
            max_perimeter_heuristic = search->getClosedShared()->getHNotClosed();
        }
    }



    // 1) Get initial abstraction
    set < int > pattern;
    for (int i = 0; i < task->get_num_goals(); i++)
        pattern.insert(task->get_goal_fact(i).var);

    cout << "Initialize initial abstraction" << endl;

    // Create first pdb with only goal variables
    // if(perimeter) {
    // best_pdb.reset(new PDBSearch(pattern, this,
    //                       originalSearch, g_timer() - generationTime, searchParams.maxStepNodes,
    //                       mgrParams, searchParams, originalStateSpace.get()));
    // } else {
    auto best_pdb = unique_ptr < PDBSearch > (new PDBSearch(pattern, this, originalStateSpace, vars, opts, task));

    best_pdb->search(generationTime, generationMemory);

    bool solved(false);
    while ((generationTime == 0 || utils::g_timer() < generationTime) &&
           (generationMemory == 0 || vars->totalMemory() < generationMemory) &&
           !best_pdb->solved()) {
        vector < unique_ptr < PDBSearch >> new_bests;
        double new_best_value = -1;

        // 2) For every possible child of the abstraction
        //    For each element interface empty partitions influencing our
        //    already chosen partitions we try to remove it and generate a new PDB
        for (int var : best_pdb->candidate_vars()) {
            // 2a) Search the child
            set < int > child_pattern(best_pdb->get_pattern());
            child_pattern.insert(var);

            // 2b) Check if it is the best child so far
            // unique_ptr<PDBSearch> new_pdb;
            // if(perimeter) {
            //  new_pdb.reset(new PDBSearch(child_pattern, this,
            //                              originalSearch, utils::g_timer() - generationTime, searchParams.maxStepNodes,
            //                              mgrParams, searchParams, originalStateSpace.get()));
            // } else {
            auto new_pdb = unique_ptr < PDBSearch > (new PDBSearch(child_pattern, this, originalStateSpace, vars, opts, task));

            new_pdb->search(generationTime, generationMemory);

            if (new_pdb->found_solution()) {
                solved = true;
                best_pdb = std::move(new_pdb);
                // cout << "Best PDB after solution found: " << *best_pdb << endl;
                new_bests.clear();

                break;
            }

            assert((int)child_pattern.size() < task->get_num_variables() ||
                   new_pdb->getLowerBound() >= new_pdb->get_search()->getF());
            if (new_pdb->average_value() > best_pdb->average_value()) {
                new_best_value = max(new_best_value, new_pdb->average_value());
                new_bests.push_back(std::move(new_pdb));
            }
        }

        if (new_bests.empty())
            break;
        new_bests.erase(std::remove_if(
                            new_bests.begin(),
                            new_bests.end(),
                            [new_best_value](unique_ptr < PDBSearch > &x) {
                                return x->average_value() < 0.999 * new_best_value;
                            }), new_bests.end());

        if (!solved && new_bests.size() > 1) {
            set < int > new_pattern;
            for (auto &pdb : new_bests) {
                new_pattern.insert(pdb->get_pattern().begin(), pdb->get_pattern().end());
            }
            // if(perimeter) {
            //  best_pdb.reset(new PDBSearch(new_pattern, this,
            //                               originalSearch, utils::g_timer() - generationTime, searchParams.maxStepNodes,
            //                               mgrParams, searchParams, originalStateSpace.get()));
            // } else {
            best_pdb = unique_ptr < PDBSearch > (new PDBSearch(new_pattern, this, originalStateSpace, vars, opts, task));

            best_pdb->search(generationTime, generationMemory);

            assert((int)new_pattern.size() < task->get_num_variables() ||
                   best_pdb->getLowerBound() >= best_pdb->get_search()->getF());

            if (!best_pdb->found_solution() && best_pdb->average_value() < new_best_value) {
                for (auto &pdb : new_bests) {
                    if (pdb->average_value() == new_best_value) {
                        best_pdb = std::move(pdb);

                        break;
                    }
                }
            }
        } else {
            best_pdb.reset(new_bests[0].release());
            cout << "New best PDB: " << *best_pdb << endl;
        }
    }

    cout << "Final pdb: " << *best_pdb << endl;

    if (best_pdb->found_solution()) {
        // For performance reasons and toa void search failures in rare edge-cases
        cout << "Problem solved during heuristic generation" << endl;
        heuristic = unique_ptr < ADD > (new ADD(std::move(best_pdb->get_cheapest_solution_ADD())));
    } else {
        // if (perimeter) heuristic.reset(new ADD(best_pdb->getHeuristic(max_perimeter_heuristic)));
        // else
        heuristic = unique_ptr < ADD > (new ADD(std::move(best_pdb->getHeuristic())));
    }
    cout << "Done initializing Gamer PDB heuristic [" << timer << "] total memory: " << vars->totalMemory() << endl << endl;

    if (!heuristic)
        cout << "Warning: heuristic could not be computed" << endl;
}


int GamerPDBsHeuristic::compute_heuristic(const State &state) {
    int *inputs = vars->getBinaryDescription(state);
    for (const BDD &bdd : notMutexBDDs) {
        if (bdd.Eval(inputs).IsZero()) {
            return DEAD_END;
        }
    }

    int res = 0;
    if (perimeter_heuristic) {
        ADD evalNode = perimeter_heuristic->Eval(inputs);
        res = Cudd_V(evalNode.getRegularNode());
        //cout << "Perimeter: " << res << endl;
        if (res < max_perimeter_heuristic) {
            if (res == -1)
                return DEAD_END;
            else
                return res;
        }
    }

    if (heuristic) {
        ADD evalNode = heuristic->Eval(inputs);
        int abs_cost = Cudd_V(evalNode.getRegularNode());

        if (abs_cost == -1)
            return DEAD_END;
        else if (abs_cost > res)
            res = abs_cost;
    }

    return res;
}


void GamerPDBsHeuristic::dump_options() const {
    cout << "Generation time: " << generationTime << endl;
    cout << "Generation memory: " << generationMemory << endl;
}

static shared_ptr < GamerPDBsHeuristic > _parse(OptionParser &parser) {
    Heuristic::add_options_to_parser(parser);
    SymbolicSearch::add_options_to_parser(parser);

    parser.add_option < std::shared_ptr < symbolic::PlanDataBase >> (
        "plan_selection", "plan selection strategy", "top_k(num_plans=1)");

    parser.add_option < int > ("generation_time", "maximum time used in heuristic generation", "1200");

    parser.add_option < double > ("generation_memory",
                                  "maximum memory used in heuristic generation", to_string(3e9));
    parser.add_option < bool > ("super_pdb", "construct super pdb", "false");

    parser.add_option < bool > ("perimeter", "construct perimeter pdbs", "false");



    Options opts = parser.parse();
    if (parser.help_mode())
        return 0;

    if (parser.dry_run()) {
        return 0;
    } else {
        return make_shared < GamerPDBsHeuristic > (opts);
    }
}


static Plugin < Evaluator > _plugin("gamer_pdbs", _parse);
}
