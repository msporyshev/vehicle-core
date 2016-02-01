#include "regul_storage.h"

#include <vector>

using namespace std;

void RegulStorage::add(shared_ptr<Regulator> regulator)
{
    vector<shared_ptr<Regulator>> new_conflicted;
    vector<shared_ptr<Regulator>> new_stored;
    for (const auto stored_r : stored) {
        if (regulator->has_conflicts(stored_r)) {
            new_conflicted.push_back(stored_r);
        } else {
            new_stored.push_back(stored_r);
        }
    }
    new_stored.push_back(regulator);
    stored = new_stored;
    conflicted = new_conflicted;
}

void RegulStorage::add(const vector<shared_ptr<Regulator>>& data)
{
    for (auto e : data) {
        add(e);
    }
}

const vector<shared_ptr<Regulator>>& RegulStorage::get_stored() const
{
    return stored;
}

const vector<shared_ptr<Regulator>>& RegulStorage::get_conflicted() const
{
    return conflicted;
}

void RegulStorage::clear_conflicted()
{
    conflicted.clear();
}

void RegulStorage::clear_stored()
{
    stored.clear();
}

void RegulStorage::clear_all()
{
    clear_stored();
    clear_conflicted();
}
