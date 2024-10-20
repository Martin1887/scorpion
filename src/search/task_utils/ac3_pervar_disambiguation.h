#ifndef TASK_UTILS_DISAMBIGUATION_H
#define TASK_UTILS_DISAMBIGUATION_H

#include "disambiguation_method.h"

namespace plugins {
class Options;
}

namespace disambiguation {
class AC3PerVarDisambiguation : public DisambiguationMethod {
    bool arc_reduce(CartesianSet &disambiguated,
                    int var,
                    int mutex_var,
                    const mutex_set_for_value &var_mutexes) const;
    void add_new_mutexes(int removed_var,
                         const std::vector<int> &var_mutex_vars,
                         std::vector<int> &worklist) const;
public:
    AC3PerVarDisambiguation(const plugins::Options &) {}
    virtual bool disambiguate(CartesianState &, const MutexInformation &) const override;
};
}
#endif
