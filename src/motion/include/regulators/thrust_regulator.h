#pragma once

#include "regulator.h"

#include <motion/CmdFixThrust.h>

#include <vector>
#include <log.h>

struct ThrustRegulConfig
{
    ThrustRegulConfig(const YamlReader& config);
    ~ThrustRegulConfig();
};

class ThrustRegulator : public Regulator
{
public:
    ThrustRegulator(motion::CmdFixThrust msg, const std::shared_ptr<ThrustRegulConfig> config);
    ~ThrustRegulator();

protected:
    virtual void update(const NavigInfo& msg) override;

private:
    Axis axis;
    double value;
};
