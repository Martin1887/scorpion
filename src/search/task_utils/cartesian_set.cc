#include "cartesian_set.h"

#include "cartesian_set_facts_proxy_iterator.h"

using namespace std;

namespace cartesian_set {
void CartesianSet::init_facts(const vector<FactPair> &facts) {
    vector<bool> reset_vars(n_vars(), false);
    empty = true;
    for (FactPair fact : facts) {
        empty = false;
        if (!reset_vars[fact.var]) {
            set_single_value(fact.var, fact.value);
            reset_vars[fact.var] = true;
        } else {
            add(fact.var, fact.value);
        }
    }
}
void CartesianSet::init_facts(const PreconditionsProxy &facts) {
    vector<bool> reset_vars(n_vars(), false);
    empty = true;
    for (FactProxy fact : facts) {
        empty = false;
        if (!reset_vars[fact.get_variable().get_id()]) {
            set_single_value(fact.get_variable().get_id(), fact.get_value());
            reset_vars[fact.get_variable().get_id()] = true;
        } else {
            add(fact.get_variable().get_id(), fact.get_value());
        }
    }
}

CartesianSet::CartesianSet(const TaskProxy &task) {
    empty = false;
    domain_subsets.reserve(task.get_variables().size());
    for (const auto &var : task.get_variables()) {
        Bitset domain(var.get_domain_size());
        domain.set();
        domain_subsets.push_back(std::move(domain));
    }
}
CartesianSet::CartesianSet(const TaskProxy &task, const vector<FactPair> &facts)
    : CartesianSet(task) {
    init_facts(facts);
}
CartesianSet::CartesianSet(const TaskProxy &task, const PreconditionsProxy &facts)
    : CartesianSet(task) {
    init_facts(facts);
}
CartesianSet::CartesianSet(const vector<int> &domain_sizes) {
    empty = false;
    domain_subsets.reserve(domain_sizes.size());
    for (int domain_size : domain_sizes) {
        Bitset domain(domain_size);
        domain.set();
        domain_subsets.push_back(move(domain));
    }
}
CartesianSet::CartesianSet(const vector<int> &domain_sizes, const vector<FactPair> &facts)
    : CartesianSet(domain_sizes) {
    init_facts(facts);
}
CartesianSet::CartesianSet(const vector<int> &domain_sizes, const PreconditionsProxy &facts)
    : CartesianSet(domain_sizes) {
    init_facts(facts);
}

int CartesianSet::n_vars() const {
    return domain_subsets.size();
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
        int domain_size = var_size(var);
        for (int value = 0; value < domain_size; ++value) {
            if (!test(var, value)) {
                intersection.remove(var, value);
            }
        }
    }
    return intersection;
}
utils::HashSet<int> CartesianSet::var_intersection(const CartesianSet &other, int var) const {
    utils::HashSet<int> values;
    int domain_size = domain_subsets[var].size();
    values.reserve(domain_size);
    for (int value = 0; value < domain_size; ++value) {
        if (test(var, value) && other.test(var, value)) {
            values.insert(value);
        }
    }
    return values;
}

int CartesianSet::count(int var) const {
    return domain_subsets[var].count();
}

int CartesianSet::var_size(int var) const {
    return domain_subsets[var].size();
}

bool CartesianSet::got_empty() {
    // TODO: Naive implementation.
    for (int var = 0; var < n_vars(); var++) {
        if (count(var) == 0) {
            empty = true;
            break;
        }
    }

    return empty;
}

bool CartesianSet::is_empty() const {
    return empty;
}

bool CartesianSet::all_values_set(int var) const {
    return count(var) == static_cast<int>(domain_subsets[var].size());
}


auto CartesianSet::get_values(int var) const -> vector<int> {
    vector<int> values(count(var), -1);
    int domain_size = var_size(var);
    int i = 0;
    for (int value = 0; value < domain_size; ++value) {
        if (test(var, value)) {
            values[i] = value;
            i++;
        }
    }
    return values;
}
utils::HashSet<int> CartesianSet::get_values_set(int var) const {
    utils::HashSet<int> values;
    int domain_size = domain_subsets[var].size();
    for (int value = 0; value < domain_size; ++value) {
        if (test(var, value)) {
            values.insert(value);
        }
    }
    return values;
}

void CartesianSet::set_values(int var, const vector<int> &values) {
    remove_all(var);
    for (int value : values) {
        add(var, value);
    }
}
void CartesianSet::set_values(int var, const utils::HashSet<int> &values) {
    remove_all(var);
    for (int value : values) {
        add(var, value);
    }
}

void CartesianSet::set_values(int var, const CartesianSet &other) {
    remove_all(var);
    for (auto &&[var, value] : other.iter(var)) {
        add(var, value);
    }
}

bool CartesianSet::intersects_intersection(const CartesianSet &other, const CartesianSet &another, int var) const {
    int len = var_size(var);
    for (int value = 0; value < len; value++) {
        if (test(var, value) && other.test(var, value) && another.test(var, value)) {
            return true;
        }
    }

    return false;
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

bool CartesianSet::is_equal_in_var(const CartesianSet &other, int var) const {
    return domain_subsets[var] == other.domain_subsets[var];
}

CartesianSetFactsProxyIterator CartesianSet::iter(int start, int end, bool inverse) const {
    return CartesianSetFactsProxyIterator(this, start, end, inverse);
}
CartesianSetFactsProxyIterator CartesianSet::iter(int start) const {
    return iter(start, start + 1);
}
CartesianSetFactsProxyIterator CartesianSet::iter() const {
    return iter(0, n_vars());
}
CartesianSetFactsProxyIterator CartesianSet::inverse_iter() const {
    return iter(0, n_vars(), true);
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
