#ifndef LANDMARKS_LANDMARK_SUM_HEURISTIC_H
#define LANDMARKS_LANDMARK_SUM_HEURISTIC_H

#include "landmark.h"
#include "landmark_heuristic.h"

namespace landmarks {
class LandmarkSumHeuristic : public LandmarkHeuristic {
    const bool dead_ends_reliable;

    std::vector<int> min_first_achiever_costs;
    std::vector<int> min_possible_achiever_costs;

    int get_min_cost_of_achievers(Achievers &achievers) const;
    void compute_landmark_costs();

    int get_heuristic_value(const State &state) override;
public:
    explicit LandmarkSumHeuristic(const plugins::Options &opts);

    virtual bool dead_ends_are_reliable() const override;
};
}

#endif
