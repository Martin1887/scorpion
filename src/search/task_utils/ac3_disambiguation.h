#ifndef TASK_UTILS_DISAMBIGUATION_H
#define TASK_UTILS_DISAMBIGUATION_H

#include "disambiguation_method.h"
namespace disambiguation {
class AC3Disambiguation : public DisambiguationMethod {
    bool arc_reduce(CartesianSet &disambiguated,
                    int var,
                    const std::tuple<int, FactPair> &mutex,
                    const std::set<std::tuple<int, FactPair>> &var_mutexes) const;
    void add_new_mutexes(const std::tuple<int, FactPair> &removed_mutex,
                         const std::set<std::tuple<int, FactPair>> &var_mutexes,
                         std::set<std::tuple<int, FactPair>> worklist) const;
public:
    bool disambiguate(CartesianState &, const MutexInformation &) const;
};
}
#endif
