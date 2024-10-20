#ifndef TASK_UTILS_PERVAR_AC3_DISAMBIGUATION_H
#define TASK_UTILS_PERVAR_AC3_DISAMBIGUATION_H

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
public:
    AC3PerVarDisambiguation(const plugins::Options &) {}
    virtual bool disambiguate(CartesianState &, const MutexInformation &) const override;
};
}
#endif
