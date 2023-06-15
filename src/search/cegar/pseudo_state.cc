#include "pseudo_state.h"

#include <unordered_set>

using namespace std;

namespace cegar {
PseudoState::PseudoState(std::size_t n_vars, std::vector<FactPair> facts)
    : values(make_shared<vector<int>>(n_vars, -1)) {
    for (FactPair f : facts) {
        get_values()[f.var] = f.value;
    }
}

bool PseudoState::is_applicable(const OperatorProxy &op) const {
    for (FactProxy cond : op.get_preconditions()) {
        if (!includes(cond)) {
            return false;
        }
    }

    return true;
}

bool PseudoState::is_backward_applicable(const OperatorProxy &op) const {
    unordered_set<int> effect_vars{};
    for (EffectProxy ef : op.get_effects()) {
        effect_vars.insert(ef.get_fact().get_variable().get_id());
        if (!includes(ef.get_fact())) {
            return false;
        }
    }
    for (FactProxy cond : op.get_preconditions()) {
        // Prevail conditions (variable is in the preconditions but not in the
        // effects) must also be checked because though its value does not
        // change in the target state it must be included.
        if (effect_vars.count(cond.get_variable().get_id()) == 0) {
            if (!includes(cond)) {
                return false;
            }
        }
    }

    return true;
}

vector<int> PseudoState::vars_not_backward_applicable(const OperatorProxy &op) const {
    vector<int> not_applicable{};
    unordered_set<int> effect_vars{};
    for (EffectProxy ef : op.get_effects()) {
        effect_vars.insert(ef.get_fact().get_variable().get_id());
        if (!includes(ef.get_fact())) {
            not_applicable.push_back(ef.get_fact().get_variable().get_id());
        }
    }
    for (FactProxy cond : op.get_preconditions()) {
        // Prevail conditions (variable is in the preconditions but not in the
        // effects) must also be checked because though its value does not
        // change in the target state it must be included.
        if (effect_vars.count(cond.get_variable().get_id()) == 0) {
            if (!includes(cond)) {
                not_applicable.push_back(cond.get_variable().get_id());
            }
        }
    }

    return not_applicable;
}

PseudoState PseudoState::get_sucessor_state(const OperatorProxy &op) const {
    assert(is_applicable(op));
    PseudoState succ(*this);

    for (EffectProxy ef : op.get_effects()) {
        succ.get_values()[ef.get_fact().get_variable().get_id()] = ef.get_fact().get_value();
    }

    return succ;
}

PseudoState PseudoState::get_backward_sucessor_state(const OperatorProxy &op) const {
    assert(is_backward_applicable(op));
    PseudoState succ(*this);

    // All effects variables are set as undefined, since they could have any
    // value in the previous (backward successor) state
    for (EffectProxy ef : op.get_effects()) {
        succ.get_values()[ef.get_fact().get_variable().get_id()] = -1;
    }

    // All preconditions of the operator are set in the successor state
    for (FactProxy fact : op.get_preconditions()) {
        succ.get_values()[fact.get_variable().get_id()] = fact.get_value();
    }

    return succ;
}
}
