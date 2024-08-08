#include "unsolvable_task.h"

namespace tasks {
    int UnsolvableTask::get_num_variables() const{
        return 1;
    }
    std::string UnsolvableTask::get_variable_name(int ) const{
        return "var_unsolvable";
    }
    int UnsolvableTask::get_variable_domain_size(int ) const{
        return 2;
    }
    int UnsolvableTask::get_variable_axiom_layer(int ) const{
        return 0;
    }
    int UnsolvableTask::get_variable_default_axiom_value(int ) const{
        return 0;
    }
    std::string UnsolvableTask::get_fact_name(const FactPair &fact) const {
        return "unsolvable-" + std::to_string(fact.value);
    }
    bool UnsolvableTask::are_facts_mutex(const FactPair &, const FactPair &) const {
        return true;
    }

    int UnsolvableTask::get_operator_cost(int , bool ) const{
        return 1;
    }
    std::string UnsolvableTask::get_operator_name(int , bool ) const {
        return "";
    }
    int UnsolvableTask::get_num_operators() const {
        return 0;
    }
    int UnsolvableTask::get_num_operator_preconditions(int , bool ) const {
        return 0;
    }
    FactPair UnsolvableTask::get_operator_precondition(int , int , bool ) const {
        return FactPair(0,0);
    }
    int UnsolvableTask::get_num_operator_effects(int , bool ) const {
        return 0;
    }
    int UnsolvableTask::get_num_operator_effect_conditions(int , int , bool ) const {
        return 0;
    }
    FactPair UnsolvableTask::get_operator_effect_condition(int , int , int , bool ) const {
        return FactPair(0,0);
    }
    FactPair UnsolvableTask::get_operator_effect(int , int , bool ) const {
        return FactPair(0,0);
    }

    int UnsolvableTask::convert_operator_index(int , const AbstractTask *) const {
        return 0;
    }

    int UnsolvableTask::get_num_axioms() const {
        return 0;

    }

    int UnsolvableTask::get_num_goals() const {
        return 1;
    }
    FactPair UnsolvableTask::get_goal_fact(int ) const {
        return FactPair(0,1);
    }

    std::vector<int> UnsolvableTask::get_initial_state_values() const {
        std::vector<int> state;
        state.push_back(0);
        return state;
    }

    void UnsolvableTask::convert_ancestor_state_values(std::vector<int> &,const AbstractTask *) const {

    }

}