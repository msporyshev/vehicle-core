#pragma once

#include "regulator.h"
#include "regul_config.h"

#include <motion/CmdFixThrust.h>

#include <vector>
#include <log.h>

struct ThrustRegulConfig: RegulConfig
{
    ThrustRegulConfig(const YamlReader& config) {}
};

class ThrustRegulator : public Regulator
{
public:
    ThrustRegulator(motion::CmdFixThrust msg, const std::shared_ptr<ThrustRegulConfig> config);

protected:
    virtual void update(const NavigInfo& msg) override;

private:
    Axis axis;
    double value;
};
