#include "cartesian_set.h"

#include "utils.h"

#include <sstream>

using namespace std;

namespace cartesian_abstractions {
vector<VariableInfo> CartesianSet::var_infos;
int CartesianSet::total_num_blocks;

CartesianSet::CartesianSet(const vector<int> &domain_sizes) {
    domains.resize(total_num_blocks, 0);
    for (size_t var = 0; var < domain_sizes.size(); ++var) {
        add_all(var);
    }
}

CartesianSet::CartesianSet(const vector<int> &domain_sizes, bool init_static_members) {
    if (init_static_members) {
        set_static_members(domain_sizes);
    }
    domains.resize(total_num_blocks, 0);
    for (size_t var = 0; var < domain_sizes.size(); ++var) {
        add_all(var);
    }
}

CartesianSet::CartesianSet(const vector<int> &domain_sizes, const vector<FactPair> &facts, bool partial_state) {
    domains.resize(total_num_blocks, 0);
    if (partial_state) {
        // Create full Cartesian set, then set facts.
        for (size_t var = 0; var < domain_sizes.size(); ++var) {
            add_all(var);
        }
        vector<bool> reset_vars(domain_sizes.size(), false);
        for (FactPair fact : facts) {
            if (!reset_vars[fact.var]) {
                set_single_value(fact.var, fact.value);
                reset_vars[fact.var] = true;
            } else {
                add(fact.var, fact.value);
            }
            add(fact.var, fact.value);
        }
    } else {
        // Create empty Cartesian set, then add facts.
        for (FactPair fact : facts) {
            add(fact.var, fact.value);
        }
    }
}

void CartesianSet::set_static_members(const vector<int> &domain_sizes) {
    var_infos.clear();
    var_infos.reserve(domain_sizes.size());
    total_num_blocks = 0;
    for (int domain_size : domain_sizes) {
        int num_blocks = BitsetMath::compute_num_blocks(domain_size);
        var_infos.emplace_back(domain_size, total_num_blocks);
        total_num_blocks += num_blocks;
    }
}

int CartesianSet::n_values(int var) const {
    return var_infos[var].domain_size;
}

void CartesianSet::add(int var, int value) {
    get_view(var).set(value);
}

void CartesianSet::remove(int var, int value) {
    get_view(var).reset(value);
}

void CartesianSet::set_single_value(int var, int value) {
    remove_all(var);
    add(var, value);
}

void CartesianSet::add_all(int var) {
    get_view(var).set();
    assert(has_full_domain(var));
}

void CartesianSet::remove_all(int var) {
    get_view(var).reset();
}

int CartesianSet::count(int var) const {
    return get_view(var).count();
}

vector<int> CartesianSet::get_values(int var) const {
    vector<int> values;
    int domain_size = var_infos[var].domain_size;
    for (int value = 0; value < domain_size; ++value) {
        if (test(var, value)) {
            values.push_back(value);
        }
    }
    return values;
}

bool CartesianSet::has_full_domain(int var) const {
    bool fast_result = get_view(var).test();
#ifndef NDEBUG
    bool result = (count(var) == var_infos[var].domain_size);
    assert(fast_result == result);
    bool slow_result = true;
    for (int value = 0; value < var_infos[var].domain_size; ++value) {
        if (!test(var, value)) {
            slow_result = false;
            break;
        }
    }
    assert(result == slow_result);
#endif
    return fast_result;
}

bool CartesianSet::intersects(const CartesianSet &other) const {
    for (int var = 0; var < get_num_variables(); ++var) {
        if (!intersects(other, var)) {
            return false;
        }
    }
    return true;
}


bool CartesianSet::is_superset_of(const CartesianSet &other) const {
    for (int var = 0; var < get_num_variables(); ++var) {
        if (!is_superset_of(other, var)) {
            return false;
        }
    }
    return true;
}
bool CartesianSet::is_superset_of(const CartesianSet &other, int var) const {
    return other.is_subset_of(*this, var);
}

double CartesianSet::compute_size() const {
    double size = 1.0;
    for (int var = 0; var < get_num_variables(); ++var) {
        size *= count(var);
    }
    return size;
}
bool CartesianSet::is_subset_of(const CartesianSet &other) const {
    int num_vars = get_num_variables();
    for (int var = 0; var < num_vars; ++var) {
        if (!is_subset_of(other, var)) {
            return false;
        }
    }
    return true;
}
bool CartesianSet::is_subset_of(const CartesianSet &other, int var) const {
    return get_view(var).is_subset_of(other.get_view(var));
}

bool CartesianSet::is_equal_in_var(const CartesianSet &other, int var) const {
    return domains[var] == other.domains[var];
}

void CartesianSet::var_union(const CartesianSet &other, int var) {
    assert(other.get_num_variables() == get_num_variables());
    int n_values = var_infos[var].domain_size;
    for (int value = 0; value < n_values; value++) {
        if (other.test(var, value)) {
            add(var, value);
        }
    }
}

void CartesianSet::set_var_values(const CartesianSet &other, int var) {
    assert(other.get_num_variables() == get_num_variables());
    remove_all(var);
    var_union(other, var);
}

ostream &operator<<(ostream &os, const CartesianSet &cartesian_set) {
    string var_sep;
    os << "<";
    for (int var = 0; var < cartesian_set.get_num_variables(); ++var) {
        const ConstBitsetView &view = cartesian_set.get_view(var);
        vector<int> values;
        for (int value = 0; value < view.size(); ++value) {
            if (view.test(value))
                values.push_back(value);
        }
        assert(!values.empty());
        if (static_cast<int>(values.size()) < view.size()) {
            os << var_sep << var << "={";
            string value_sep;
            for (int value : values) {
                os << value_sep << value;
                value_sep = ",";
            }
            os << "}";
            var_sep = ",";
        }
    }
    return os << ">";
}

bool CartesianSet::operator==(const CartesianSet &other) const {
    int num_vars = get_num_variables();
    if (num_vars != other.get_num_variables()) {
        return false;
    }
    for (int var = 0; var < num_vars; var++) {
        if (!is_equal_in_var(other, var)) {
            return false;
        }
    }

    return true;
}

void CartesianSet::feed(utils::HashState &hash_state) const {
    int num_vars = get_num_variables();
    for (int var = 0; var < num_vars; var++) {
        utils::feed(hash_state, domains[var]);
    }
}
}
