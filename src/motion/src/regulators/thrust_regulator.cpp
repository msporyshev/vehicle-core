#include "regulators/thrust_regulator.h"

#include "registry.h"

using motion::CmdFixThrust;

ThrustRegulator::ThrustRegulator(CmdFixThrust msg, const std::shared_ptr<ThrustRegulConfig> config):
    Regulator(msg.id, {static_cast<Axis>(msg.axis)}, msg.timeout),
    value(msg.value)
{
    axis = static_cast<Axis>(msg.axis);
}

void ThrustRegulator::update(const NavigInfo& msg)
{
    set_thrusts({{axis, value}});
    set_success(false); // Успех этого регулятора -- это истечение таймаута
}

REG_REGUL(thrust, ThrustRegulator, CmdFixThrust, ThrustRegulConfig);
