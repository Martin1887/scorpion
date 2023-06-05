#ifndef CEGAR_PSEUDO_STATE_H
#define CEGAR_PSEUDO_STATE_H

#include "cartesian_set.h"
#include "types.h"

#include "../task_proxy.h"

#include <vector>

namespace cegar {

// Class used to search flaws in the concrete state space from goals.
// Since goals can be partially defined, some of the variables can have
// undefined value (-1) and the class provides methods to apply operators
// in these situations.
class PseudoState {
    std::shared_ptr<std::vector<int>> values;

public:
    PseudoState(std::size_t n_vars, std::vector<FactPair> facts);
    PseudoState(const PseudoState &other)
        : values(std::make_shared<std::vector<int>>(other.get_values())) {};
    PseudoState &operator=(const PseudoState &other) {
        if (this == &other) {
            return *this;
        }

        this->values = std::make_shared<std::vector<int>>(other.get_values());

        return *this;
    }
    ~PseudoState() {
        values.reset();
    }

    std::vector<int> &get_values() const {
        return *values;
    };

    bool includes(const FactProxy &fact) const {
        int fact_var = fact.get_variable().get_id();
        return get_values()[fact_var] == -1 || get_values()[fact_var] == fact.get_value();
    }

    bool includes(const State &state) const {
        for (FactProxy fact : state) {
            if (!includes(fact)) {
                return false;
            }
        }

        return true;
    }

    friend std::ostream &operator<<(std::ostream &os, const PseudoState &state) {
        std::vector<int> vals(state.get_values());
        std::string str_values;
        bool first = true;
        for (int val : vals) {
            if (!first) {
                str_values.append(", ");
            }
            str_values.append(std::to_string(val));
            first = false;
        }
        return os << "PseudoState(" << str_values << ")";
    }

    bool is_applicable(const OperatorProxy &op) const;
    PseudoState get_sucessor_state(const OperatorProxy &op) const;

    bool is_backward_applicable(const OperatorProxy &op) const;
    std::vector<int> vars_not_backward_applicable(const OperatorProxy &op) const;

    PseudoState get_backward_sucessor_state(const OperatorProxy &op) const;
};

}

#endif
