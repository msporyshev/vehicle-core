#include "regulators/thrust_regulator.h"

#include "registry.h"

using motion::CmdFixThrust;

ThrustRegulConfig::ThrustRegulConfig(const YamlReader& config)
{

}

ThrustRegulConfig::~ThrustRegulConfig()
{

}

ThrustRegulator::ThrustRegulator(CmdFixThrust msg, const std::shared_ptr<ThrustRegulConfig> config):
    Regulator(msg.id, {static_cast<Axis>(msg.axis)}, msg.timeout),
    value(msg.value)
{
    axis = static_cast<Axis>(msg.axis);
}

ThrustRegulator::~ThrustRegulator()
{

}

void ThrustRegulator::update(const NavigInfo& msg)
{
    set_thrusts({{axis, value}});
    set_success(true);
}

REG_REGUL(thrust, ThrustRegulator, CmdFixThrust, ThrustRegulConfig);
