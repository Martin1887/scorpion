#ifndef COST_SATURATION_SCP_GENERATOR_H
#define COST_SATURATION_SCP_GENERATOR_H

#include "types.h"

#include <memory>
#include <vector>

class TaskProxy;

namespace options {
class Options;
class OptionParser;
}

namespace cost_saturation {
class Abstraction;


class SCPGenerator {
protected:
    const int max_orders;
    const double max_time;
    const bool diversify;

    virtual void initialize(
        const TaskProxy &task_proxy,
        const std::vector<std::unique_ptr<Abstraction>> &abstractions,
        const std::vector<int> &costs);

    virtual CostPartitioning get_next_cost_partitioning(
        const TaskProxy &task_proxy,
        const std::vector<std::unique_ptr<Abstraction>> &abstractions,
        const std::vector<int> &costs) = 0;

    virtual bool has_next_cost_partitioning() const {
        return true;
    }

public:
    SCPGenerator(const options::Options &opts);
    virtual ~SCPGenerator() = default;

    virtual CostPartitionings get_cost_partitionings(
        const TaskProxy &task_proxy,
        const std::vector<std::unique_ptr<Abstraction>> &abstractions,
        const std::vector<int> &costs);
};


extern std::vector<std::vector<int>> compute_saturated_cost_partitioning(
    const std::vector<std::unique_ptr<Abstraction>> &abstractions,
    const std::vector<int> &order,
    const std::vector<int> &costs,
    bool debug = false);

extern void add_common_scp_generator_options_to_parser(
    options::OptionParser &parser);
}

#endif
