#ifndef COST_SATURATION_CARTESIAN_ABSTRACTION_GENERATOR_H
#define COST_SATURATION_CARTESIAN_ABSTRACTION_GENERATOR_H

#include "abstraction_generator.h"

#include <memory>
#include <vector>

namespace options {
class Options;
}

namespace cegar {
class SubtaskGenerator;
}

namespace utils {
class CountdownTimer;
class RandomNumberGenerator;
}

namespace cost_saturation {
class CartesianAbstractionGenerator : public AbstractionGenerator {
    const std::vector<std::shared_ptr<cegar::SubtaskGenerator>> subtask_generators;
    const int max_states;
    const int max_transitions;
    const double max_time;
    const bool prune_unreachable_transitions;
    const std::shared_ptr<utils::RandomNumberGenerator> rng;
    const bool debug;

    int num_states;
    int num_transitions;

    void build_abstractions_for_subtasks(
        const std::vector<std::shared_ptr<AbstractTask>> &subtasks,
        const utils::CountdownTimer &timer,
        Abstractions &abstractions);

public:
    explicit CartesianAbstractionGenerator(const options::Options &opts);

    Abstractions generate_abstractions(
        const std::shared_ptr<AbstractTask> &task);
};
}

#endif
