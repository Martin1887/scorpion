#ifndef SYMBOLIC_SYM_PDB_H
#define SYMBOLIC_SYM_PDB_H

#include "sym_state_space_manager.h"
#include "sym_variables.h"
#include <set>

namespace symbolic {
class OriginalStateSpace;

class SymPDB : public SymStateSpaceManager {
    BDD nonRelVarsCube;         // Cube BDD representing relevantVars
    BDD nonRelVarsCubeWithPrimes; // Cube BDD representing relevantVars
    std::string abstractionName;

public:
    SymPDB(const OriginalStateSpace &parent, const std::set<int> &relVars);

    virtual ~SymPDB() {}

    inline bool isRelevantVar(int var) const {
        return relevant_vars.count(var) > 0;
    }

    virtual BDD shrinkExists(const BDD &bdd, int maxNodes) const override;
    virtual BDD shrinkForall(const BDD &bdd, int maxNodes) const override;
    virtual BDD shrinkTBDD(const BDD &tBDD, int maxNodes) const override;

    /* virtual ADD getExplicitHeuristicADD(bool fw) override; */
    /* virtual void getExplicitHeuristicBDD(bool fw, std::map<int, BDD> &res)
     * override; */

    virtual std::string tag() const override;

    virtual void print(std::ostream &os, bool fullInfo) const override;
};
} // namespace symbolic

#endif
