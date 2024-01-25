#ifndef OPEN_LISTS_STANDARD_SCALAR_OPEN_LIST_H
#define OPEN_LISTS_STANDARD_SCALAR_OPEN_LIST_H

#include "../open_list_factory.h"
#include "../plugins/options.h"

namespace plugins {
class Options;
}

/*
  Open list indexed by a single int, using FIFO tie-breaking.

  Implemented as a map from int to deques.
*/

namespace standard_scalar_open_list {
class StandardScalarOpenListFactory : public OpenListFactory {
    plugins::Options options;
public:
    explicit StandardScalarOpenListFactory(const plugins::Options &options);
    virtual ~StandardScalarOpenListFactory() override = default;

    virtual std::unique_ptr<StateOpenList> create_state_open_list() override;
    virtual std::unique_ptr<EdgeOpenList> create_edge_open_list() override;
};
}

#endif
