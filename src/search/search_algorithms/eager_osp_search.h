#ifndef SEARCH_ENGINES_EAGER_OSP_SEARCH_H
#define SEARCH_ENGINES_EAGER_OSP_SEARCH_H

#include "../symbolic/sym_variables.h"
#include "eager_search.h"

class Evaluator;
class PruningMethod;

namespace plugins {
class Options;
}

namespace eager_search {
class EagerOspSearch : public EagerSearch {
protected:
    symbolic::SymVariables vars;
    ADD add_utility_function;
    double max_utility;

    State best_state;
    double best_utility;

    virtual void initialize() override;
    virtual SearchStatus step() override;
    virtual std::optional<SearchNode> fetch_next_node() override;

public:
    explicit EagerOspSearch(const plugins::Options &opts);
    virtual ~EagerOspSearch() = default;
};
} // namespace eager_search

#endif
