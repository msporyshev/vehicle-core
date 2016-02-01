#pragma once

#include "regulators/regulator.h"

#include <vector>
#include <memory>

class RegulStorage
{
public:
    void add(std::shared_ptr<Regulator> regulator);
    void add(const std::vector<std::shared_ptr<Regulator>>& data);

    const std::vector<std::shared_ptr<Regulator>>& get_stored() const;
    const std::vector<std::shared_ptr<Regulator>>& get_conflicted() const;

    void clear_all();
    void clear_stored();
    void clear_conflicted();
private:
    std::vector<std::shared_ptr<Regulator>> stored;
    std::vector<std::shared_ptr<Regulator>> conflicted;
};
