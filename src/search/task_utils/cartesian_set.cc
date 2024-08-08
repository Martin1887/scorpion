#include "cartesian_set.h"

#include <sstream>

using namespace std;

namespace cartesian_set {
CartesianSet::CartesianSet(const vector<int> &domain_sizes) {
    domain_subsets.reserve(domain_sizes.size());
    for (int domain_size : domain_sizes) {
        Bitset domain(domain_size);
        domain.set();
        domain_subsets.push_back(move(domain));
    }
}
CartesianSet::CartesianSet(const vector<int> &domain_sizes, vector<FactPair> facts)
    : CartesianSet(domain_sizes) {
    vector<bool> reset_vars(domain_sizes.size(), false);
    for (FactPair fact : facts) {
        if (!reset_vars[fact.var]) {
            set_single_value(fact.var, fact.value);
            reset_vars[fact.var] = true;
        } else {
            add(fact.var, fact.value);
        }
    }
}

int CartesianSet::n_vars() const {
    return domain_subsets.size();
}

int CartesianSet::n_values(int var) const {
    return domain_subsets[var].size();
}

void CartesianSet::add(int var, int value) {
    domain_subsets[var].set(value);
}

void CartesianSet::remove(int var, int value) {
    domain_subsets[var].reset(value);
}

void CartesianSet::set_single_value(int var, int value) {
    remove_all(var);
    add(var, value);
}

void CartesianSet::add_all(int var) {
    domain_subsets[var].set();
}

void CartesianSet::remove_all(int var) {
    domain_subsets[var].reset();
}

CartesianSet CartesianSet::intersection(const CartesianSet &other) const {
    CartesianSet intersection(other);
    int num_vars = domain_subsets.size();
    for (int var = 0; var < num_vars; ++var) {
        int domain_size = domain_subsets[var].size();
        for (int value = 0; value < domain_size; ++value) {
            if (!test(var, value)) {
                intersection.remove(var, value);
            }
        }
    }
    return intersection;
}

int CartesianSet::count(int var) const {
    return domain_subsets[var].count();
}

bool CartesianSet::all_values_set(int var) const {
    return count(var) == (int)domain_subsets[var].size();
}

vector<int> CartesianSet::get_values(int var) const {
    vector<int> values;
    int domain_size = domain_subsets[var].size();
    for (int value = 0; value < domain_size; ++value) {
        if (test(var, value)) {
            values.push_back(value);
        }
    }
    return values;
}

bool CartesianSet::intersects(const CartesianSet &other, int var) const {
    return domain_subsets[var].intersects(other.domain_subsets[var]);
}

bool CartesianSet::intersects(const CartesianSet &other) const {
    int num_vars = domain_subsets.size();
    for (int var = 0; var < num_vars; ++var) {
        if (!intersects(other, var)) {
            return false;
        }
    }
    return true;
}

bool CartesianSet::is_superset_of(const CartesianSet &other) const {
    int num_vars = domain_subsets.size();
    for (int var = 0; var < num_vars; ++var) {
        if (!other.domain_subsets[var].is_subset_of(domain_subsets[var]))
            return false;
    }
    return true;
}

ostream &operator<<(ostream &os, const CartesianSet &cartesian_set) {
    int num_vars = cartesian_set.domain_subsets.size();
    string var_sep;
    os << "<";
    for (int var = 0; var < num_vars; ++var) {
        const Bitset &domain = cartesian_set.domain_subsets[var];
        vector<int> values;
        for (size_t value = 0; value < domain.size(); ++value) {
            if (domain[value])
                values.push_back(value);
        }
        assert(!values.empty());
        if (values.size() < domain.size()) {
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
    if (n_vars() != other.n_vars()) {
        return false;
    }
    for (int var = 0; var < n_vars(); var++) {
        if (get_values(var) != other.get_values(var)) {
            return false;
        }
    }

    return true;
}
}
