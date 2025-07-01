#ifndef TASK_UTILS_DISAMBIGUATION_METHOD_H
#define TASK_UTILS_DISAMBIGUATION_METHOD_H

#include "cartesian_state.h"
#include "mutex_information.h"

#include "../plugins/options.h"

using namespace cartesian_state;

namespace plugins {
class Options;
}

namespace utils {
struct HashMutexVars {
    std::size_t operator()(const cartesian_set::CartesianSet &val) const {
        HashState hash_state;
        val.feed_vars_with_mutexes(hash_state);
        return hash_state.get_hash64();
    }
};
}

namespace disambiguation  {
class DisambiguationMethod {
    bool cache_disambiguations = false;
    phmap::flat_hash_map<CartesianSet, std::vector<FactPair>, utils::HashMutexVars> cache;
    std::vector<FactPair> get_disambiguation_removed_values(const CartesianState &cartesian_set,
                                                            const MutexInformation &mutex_information);
public:
    DisambiguationMethod(bool cache_disambiguations)
        : cache_disambiguations(cache_disambiguations) {}
    virtual ~DisambiguationMethod() = default;

    virtual std::vector<FactPair> disambiguation_removed_facts(CartesianState &, const MutexInformation &);

    virtual CartesianState disambiguate_copy(const CartesianState &, const MutexInformation &, std::optional<int> var = std::nullopt) const;

    // Returns true if the Cartesian set is changed
    virtual bool disambiguate(CartesianState &, const MutexInformation &, std::optional<int> var = std::nullopt) const = 0;
    virtual bool test_disambiguate(const CartesianState &, const MutexInformation &, int var, const std::set<int> &values_for_var) const = 0;

    static void add_disambiguation_base_options(plugins::Feature &feature);
};

class NoDisambiguation : public DisambiguationMethod {
public:
    NoDisambiguation(const plugins::Options &opt)
        : DisambiguationMethod(opt.get<bool>("cache_disambiguations")) {}

    virtual bool disambiguate(CartesianState &, const MutexInformation &, std::optional<int>) const override {
        return false;
    }

    // Test if the state can be disambiguated without actually disambiguating it.
    // Faster because it should early return when a disambiguation is asserted.
    virtual bool test_disambiguate(const CartesianState &, const MutexInformation &, int, const std::set<int> &) const override {
        return false;
    }
};
}

#endif
