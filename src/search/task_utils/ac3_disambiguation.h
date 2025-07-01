#ifndef TASK_UTILS_AC3_DISAMBIGUATION_H
#define TASK_UTILS_AC3_DISAMBIGUATION_H

#include "disambiguation_method.h"

namespace plugins {
class Options;
}

namespace disambiguation {
class AC3Disambiguation : public DisambiguationMethod {
    bool arc_reduce(CartesianSet &disambiguated,
                    int var,
                    int mutex_var,
                    const mutex_set_for_value &var_mutexes) const;
    void add_new_mutexes(int current_var,
                         int removed_var,
                         const std::vector<int> &var_mutex_vars,
                         vars_pair_queue &worklist) const;
    bool test_arc_reduce(const CartesianSet &disambiguated,
                         const std::set<int> &values_for_var,
                         int mutex_var,
                         const mutex_set_for_value &var_mutexes) const;
public:
    AC3Disambiguation(const plugins::Options &opt)
        : DisambiguationMethod(opt.get<bool>("cache_disambiguations")) {}
    virtual bool disambiguate(CartesianState &, const MutexInformation &, std::optional<int> var) const override;
    virtual bool test_disambiguate(const CartesianState &, const MutexInformation &, int var, const std::set<int> &values_for_var) const override;
};
}
#endif
