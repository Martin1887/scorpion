#ifndef CARTESIAN_ABSTRACTIONS_SUBTASK_GENERATORS_H
#define CARTESIAN_ABSTRACTIONS_SUBTASK_GENERATORS_H

#include "flaw_search.h"

#include "split_selector.h"

#include "../plugins/options.h"

#include <memory>
#include <vector>

class AbstractTask;
struct FactPair;

namespace landmarks {
class LandmarkGraph;
}

namespace plugins {
class Options;
}

namespace utils {
class RandomNumberGenerator;
class LogProxy;
}

namespace cartesian_abstractions {
using Facts = std::vector<FactPair>;

struct Subtask {
    // Subtasks can be copies of the same task/subtask (with different split
    // strategies for instance) or subtasks of the original Planning task. Each
    // subproblem identifies a subtask of the original Planning task, so
    // landmarks and potentials are the same for all subtasks of the same
    // subproblem.
    int subproblem_id;
    std::shared_ptr<AbstractTask> subtask;
    PickFlawedAbstractState pick_flawed_abstract_state;
    PickSplit pick_split;
    PickSplit tiebreak_split;
    PickSequenceFlaw sequence_split;
    PickSequenceFlaw sequence_tiebreak_split;
    bool intersect_flaw_search_abstract_states;
};
using SharedTasks = std::vector<Subtask>;

enum class FactOrder {
    ORIGINAL,
    RANDOM,
    HADD_UP,
    HADD_DOWN
};


Facts filter_and_order_facts(
    const std::shared_ptr<AbstractTask> &task,
    FactOrder fact_order,
    Facts &facts,
    utils::RandomNumberGenerator &rng,
    utils::LogProxy &log);

/*
  Create focused subtasks.
*/
class SubtaskGenerator {
protected:
    PickFlawedAbstractState pick_flawed_abstract_state;
    PickSplit pick_split;
    PickSplit tiebreak_split;
    PickSequenceFlaw sequence_split;
    PickSequenceFlaw sequence_tiebreak_split;
    bool intersect_flaw_search_abstract_states;
public:
    virtual SharedTasks get_subtasks(
        const std::shared_ptr<AbstractTask> &task,
        utils::LogProxy &log) const = 0;
    SubtaskGenerator(const plugins::Options &opts)
        : pick_flawed_abstract_state(opts.get<PickFlawedAbstractState>("pick_flawed_abstract_state")),
          pick_split(opts.get<PickSplit>("pick_split")),
          tiebreak_split(opts.get<PickSplit>("tiebreak_split")),
          sequence_split(opts.get<PickSequenceFlaw>("sequence_split")),
          sequence_tiebreak_split(opts.get<PickSequenceFlaw>("sequence_tiebreak_split")),
          intersect_flaw_search_abstract_states(opts.get<bool>("intersect_flaw_search_abstract_states")) {
    }
    virtual ~SubtaskGenerator() = default;
};


/*
  Return copies of the original task.
*/
class TaskDuplicator : public SubtaskGenerator {
    int num_copies;

public:
    explicit TaskDuplicator(const plugins::Options &opts);

    virtual SharedTasks get_subtasks(
        const std::shared_ptr<AbstractTask> &task,
        utils::LogProxy &log) const override;
};


/*
  Use ModifiedGoalsTask to return a subtask for each goal fact.
*/
class GoalDecomposition : public SubtaskGenerator {
    FactOrder fact_order;
    std::shared_ptr<utils::RandomNumberGenerator> rng;

public:
    explicit GoalDecomposition(const plugins::Options &opts);

    virtual SharedTasks get_subtasks(
        const std::shared_ptr<AbstractTask> &task,
        utils::LogProxy &log) const override;
};


/*
  Nest ModifiedGoalsTask and DomainAbstractedTask to return subtasks
  focussing on a single landmark fact.
*/
class LandmarkDecomposition : public SubtaskGenerator {
    FactOrder fact_order;
    bool combine_facts;
    std::shared_ptr<utils::RandomNumberGenerator> rng;

    /* Perform domain abstraction by combining facts that have to be
       achieved before a given landmark can be made true. */
    std::shared_ptr<AbstractTask> build_domain_abstracted_task(
        const std::shared_ptr<AbstractTask> &parent,
        const landmarks::LandmarkGraph &landmark_graph,
        const FactPair &fact) const;

public:
    explicit LandmarkDecomposition(const plugins::Options &opts);

    virtual SharedTasks get_subtasks(
        const std::shared_ptr<AbstractTask> &task,
        utils::LogProxy &log) const override;
};
}

#endif
