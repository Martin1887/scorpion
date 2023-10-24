#ifndef CEGAR_SPLIT_SELECTOR_H
#define CEGAR_SPLIT_SELECTOR_H

#include "abstract_state.h"

#include "../task_proxy.h"

#include "../utils/logging.h"

#include <memory>
#include <vector>

namespace additive_heuristic {
class AdditiveHeuristic;
}

namespace utils {
class RandomNumberGenerator;
}

namespace cegar {
class AbstractState;
struct Split;

// Strategies for selecting a split in case there are multiple possibilities.
enum class PickSplit {
    RANDOM,
    // Number of values that land in the state whose h-value is probably raised.
    MIN_UNWANTED,
    MAX_UNWANTED,
    // Refinement: - (remaining_values / original_domain_size)
    MIN_REFINED,
    MAX_REFINED,
    // Compare the h^add(s_0) values of the facts.
    MIN_HADD,
    MAX_HADD,
    // Position in partial ordering of causal graph.
    MIN_CG,
    MAX_CG,
    // Compute split that covers the maximum number of flaws for several concrete states.
    MAX_COVER
};


struct Split {
    int count;
    int abstract_state_id;
    int var_id;
    int value;
    std::vector<int> values;

    Split(int abstract_state_id, int var_id, int value, std::vector<int> &&values, int count)
        : count(count),
          abstract_state_id(abstract_state_id),
          var_id(var_id),
          value(value),
          values(move(values)) {
        assert(count >= 1);
    }

    bool combine_with(Split &&other);

    bool operator==(const Split &other) const {
        assert(var_id == other.var_id);
        if (value == other.value) {
            return values == other.values;
        } else if (values.size() == 1 && other.values.size() == 1) {
            // If we need to separate exactly two values, their order doesn't matter.
            return value == other.values[0] && other.value == values[0];
        } else {
            return false;
        }
    }

    friend std::ostream &operator<<(std::ostream &os, const Split &s) {
        return os << "<" << s.var_id << "=" << s.value << "|" << s.values
                  << ":" << s.count << ">";
    }
};

struct SplitProperties {
    std::unique_ptr<Split> split;
    bool backward_direction;
    int n_forward_flaws;
    int n_backward_flaws;

    SplitProperties(std::unique_ptr<Split> split,
                    bool backward_direction,
                    int n_forward_flaws = 0,
                    int n_backward_flaws = 0)
        : split(std::move(split)),
          backward_direction(backward_direction),
          n_forward_flaws(n_forward_flaws),
          n_backward_flaws(n_backward_flaws) {}
};

struct SplitAndAbsState {
    std::unique_ptr<Split> split;
    const AbstractState &abs;

    friend std::ostream &operator<<(std::ostream &os, const SplitAndAbsState &s) {
        if (s.split) {
            return os << "[" << *s.split << " in " << s.abs.get_id() << "]";
        } else {
            return os << "nullptr split";
        }
    }
};

/*
  Select split in case there are multiple possible splits.
*/
class SplitSelector {
    const std::shared_ptr<AbstractTask> task;
    const TaskProxy task_proxy;
    const bool debug;
    std::unique_ptr<additive_heuristic::AdditiveHeuristic> additive_heuristic;

    const PickSplit first_pick;
    const PickSplit tiebreak_pick;

    int get_num_unwanted_values(const AbstractState &state, const Split &split) const;
    double get_refinedness(const AbstractState &state, int var_id) const;
    int get_hadd_value(int var_id, int value) const;
    int get_min_hadd_value(int var_id, const std::vector<int> &values) const;
    int get_max_hadd_value(int var_id, const std::vector<int> &values) const;

    double rate_split(const AbstractState &state, const Split &split, PickSplit pick) const;
    std::vector<Split> compute_max_cover_splits(
        std::vector<std::vector<Split>> &&splits) const;
    Split select_from_best_splits(
        const AbstractState &abstract_state,
        std::vector<Split> &&splits,
        utils::RandomNumberGenerator &rng) const;
    SplitAndAbsState select_from_best_splits(
        std::vector<SplitAndAbsState> &&splits,
        utils::RandomNumberGenerator &rng) const;
    SplitProperties select_from_sequence_splits(
        std::vector<SplitAndAbsState> &&forward_splits,
        std::vector<SplitAndAbsState> &&backward_splits,
        utils::RandomNumberGenerator &rng) const;
    std::vector<Split> reduce_to_best_splits(
        const AbstractState &abstract_state,
        std::vector<std::vector<Split>> &&splits) const;

public:
    SplitSelector(
        const std::shared_ptr<AbstractTask> &task,
        PickSplit pick,
        PickSplit tiebreak_pick,
        bool debug);
    ~SplitSelector();

    Split pick_split(
        const AbstractState &abstract_state,
        std::vector<std::vector<Split>> &&splits,
        utils::RandomNumberGenerator &rng) const;

    SplitProperties pick_sequence_split(
        std::vector<SplitAndAbsState> &&forward_splits,
        std::vector<SplitAndAbsState> &&backward_splits,
        utils::RandomNumberGenerator &rng) const;
};
}

#endif
