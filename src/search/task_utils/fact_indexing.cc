#include "fact_indexing.h"

FactIndexing::FactIndexing(const AbstractTask & task) {
    p_index.resize(task.get_num_variables());
    for (int i = 0; i < task.get_num_variables(); i++) {
        p_index[i].reserve(task.get_variable_domain_size(i));
        for (int j = 0; j < task.get_variable_domain_size(i); j++) {
            p_index_reverse.emplace_back(i, j);
            p_index[i].emplace_back(num_facts++);
        }
    }
}