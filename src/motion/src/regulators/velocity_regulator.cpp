#include "regulators/velocity_regulator.h"
#include "registry.h"

using namespace std;

using motion::CmdFixVelocity;

VelocityRegulConfig::VelocityRegulConfig(const YamlReader& config): PidRegulConfig(config)
{
    config.SET_PARAM(accuracy);
}

VelocityRegulator::VelocityRegulator(motion::CmdFixVelocity msg, shared_ptr<const VelocityRegulConfig> config):
    Regulator(msg.id, {Axis::TX}, msg.timeout),
    controller(config->kp, config->ki, config->kd),
    target_velocity(msg.value),
    config(config)
{
    set_log_headers({"vel_w", "vel_r"});
}

VelocityRegulator::~VelocityRegulator()
{

}

void VelocityRegulator::update(const NavigInfo& msg)
{
    double err = target_velocity - msg.velocity_forward;
    double thrust = controller.update(err);
    set_success(err, config->accuracy);
    set_thrusts({{Axis::TX, thrust}});
    set_log_values({target_velocity, msg.velocity_forward});
}

REG_REGUL(velocity, VelocityRegulator, CmdFixVelocity, VelocityRegulConfig);
