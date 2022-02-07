#ifndef PDBS_PATTERN_GENERATOR_RANDOM_H
#define PDBS_PATTERN_GENERATOR_RANDOM_H

#include "pattern_generator.h"

namespace utils {
class RandomNumberGenerator;
}

namespace pdbs {
class PatternGeneratorRandom : public PatternGenerator {
    const int max_pdb_size;
    const int max_time;
    const bool bidirectional;
    std::shared_ptr<utils::RandomNumberGenerator> rng;
public:
    explicit PatternGeneratorRandom(options::Options &opts);

    virtual PatternInformation generate(
        const std::shared_ptr<AbstractTask> &task) override;
};
}

#endif
