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

    bool test_arc_reduce(const CartesianSet &disambiguated,
                         const std::set<int> &values_for_mutex_var,
                         int var,
                         int mutex_var,
                         const mutex_set_for_value &var_mutexes) const;
public:
    AC3PerVarDisambiguation(const plugins::Options &opt)
        : DisambiguationMethod(opt.get<bool>("cache_disambiguations")) {}
    virtual bool disambiguate(CartesianState &, const MutexInformation &, std::optional<int> var) const override;
    virtual bool test_disambiguate(const CartesianState &, const MutexInformation &, int mutex_var, const std::set<int> &values_for_var) const override;
};
}
#endif
