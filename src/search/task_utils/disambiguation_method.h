#ifndef TASK_UTILS_DISAMBIGUATION_METHOD_H
#define TASK_UTILS_DISAMBIGUATION_METHOD_H

#include "cartesian_state.h"
#include "mutex_information.h"

using namespace cartesian_state;

namespace disambiguation  {
class DisambiguationMethod {
public:
    DisambiguationMethod() = default;
    virtual ~DisambiguationMethod() = default;

    virtual CartesianState disambiguate_copy(const CartesianState &, const MutexInformation &) const;

    // Returns true if the cartesian set is changed
    virtual bool disambiguate(CartesianState &, const MutexInformation &) const = 0;
};

class NoDisambiguation : public DisambiguationMethod {
public:
    virtual bool disambiguate(CartesianState &, const MutexInformation &) const override {
        return false;
    }
};
}

#endif
