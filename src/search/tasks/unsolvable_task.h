#ifndef TASKS_UNSOLVABLE_TASK_H
#define TASKS_UNSOLVABLE_TASK_H

#include "../abstract_task.h"
namespace tasks {
    class UnsolvableTask : public AbstractTask {

        virtual int get_num_variables() const;
        virtual std::string get_variable_name(int var) const;
        virtual int get_variable_domain_size(int var) const;
        virtual int get_variable_axiom_layer(int var) const;
        virtual int get_variable_default_axiom_value(int var) const;
        virtual std::string get_fact_name(const FactPair &fact) const;
        virtual bool are_facts_mutex(const FactPair &fact1, const FactPair &fact2) const;

        virtual int get_operator_cost(int index, bool is_axiom) const;
        virtual std::string get_operator_name(int index, bool is_axiom) const ;
        virtual int get_num_operators() const;
        virtual int get_num_operator_preconditions(int index, bool is_axiom) const;
        virtual FactPair get_operator_precondition(int op_index, int fact_index, bool is_axiom) const;
        virtual int get_num_operator_effects(int op_index, bool is_axiom) const;
        virtual int get_num_operator_effect_conditions(int op_index, int eff_index, bool is_axiom) const;
        virtual FactPair get_operator_effect_condition(int op_index, int eff_index, int cond_index, bool is_axiom) const;
        virtual FactPair get_operator_effect(int op_index, int eff_index, bool is_axiom) const;

        virtual int convert_operator_index(int index, const AbstractTask *ancestor_task) const;

        virtual int get_num_axioms() const;

        virtual int get_num_goals() const;
        virtual FactPair get_goal_fact(int index) const;

        virtual std::vector<int> get_initial_state_values() const;

        virtual void convert_ancestor_state_values(std::vector<int> &values,const AbstractTask *ancestor_task) const;
    };
}

#endif