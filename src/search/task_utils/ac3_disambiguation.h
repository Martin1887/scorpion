#ifndef TASK_UTILS_DISAMBIGUATION_H
#define TASK_UTILS_DISAMBIGUATION_H

#include "disambiguation_method.h"

namespace plugins {
class Options;
}

namespace disambiguation {
class AC3Disambiguation : public DisambiguationMethod {
    bool arc_reduce(CartesianSet &disambiguated,
                    int var,
                    const std::tuple<int, FactPair> &mutex,
                    const mutex_set_for_value &var_mutexes) const;
    void add_new_mutexes(const std::tuple<int, FactPair> &removed_mutex,
                         const mutex_set_for_value &var_mutexes,
                         mutex_set_for_value &worklist) const;
public:
    AC3Disambiguation(const plugins::Options &) {}
    virtual bool disambiguate(CartesianState &, MutexInformation &) const override;
};
}
#endif
