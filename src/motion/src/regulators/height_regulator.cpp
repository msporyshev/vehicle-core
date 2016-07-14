#include "regulators/height_regulator.h"

#include "registry.h"

using namespace std;

using motion::CmdFixHeight;

HeightRegulConfig::HeightRegulConfig(const YamlReader& config): PidRegulConfig(config)
{
    config.SET_PARAM(buoyancy_thrust);

    config.SET_PARAM(accuracy);
    bound_vel = config.is_param_readable(max_finishing_vel, "max_finishing_vel");

}

HeightRegulator::HeightRegulator(CmdFixHeight msg, shared_ptr<const HeightRegulConfig> config):
    Regulator(msg.id, {Axis::TZ}, msg.timeout),
    controller(config->kp, config->ki, config->kd),
    cmd_height(msg.value),
    coord_system(static_cast<CoordSystem>(msg.coord_system)),
    config(config)
{

}

void HeightRegulator::initialize(const NavigInfo& msg)
{
    if (coord_system == CoordSystem::ABS) {
        target_height = cmd_height;
    } else {
        target_height = cmd_height + msg.height;
    }
    set_log_headers({"dpth_w", "dpth_r"});
}

void HeightRegulator::update(const NavigInfo& msg)
{
    double err = target_height - msg.height;
    double err_d = -msg.velocity_height;
    double thrust = controller.update(err, err_d) + config->buoyancy_thrust;
    if (config->bound_vel) {
        set_success(fabs(err) < config->accuracy && fabs(msg.velocity_height) < config->max_finishing_vel);
    } else {
        set_success(fabs(err) < config->accuracy);
    }
    set_thrusts({{Axis::TZ, thrust}});
    set_log_values({target_height, msg.height});
}

REG_REGUL(height, HeightRegulator, CmdFixHeight, HeightRegulConfig);
