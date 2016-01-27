#include "regulators/depth_regulator.h"

#include "registry.h"

using namespace std;

using motion::CmdFixDepth;
using motion::CmdFixDepthConf;

DepthRegulConfig::DepthRegulConfig(const YamlReader& config)
{
    config.SET_PARAM(kp);
    config.SET_PARAM(ki);
    config.SET_PARAM(kd);

    config.SET_PARAM(buoyancy_thrust);

    config.SET_PARAM(accuracy);
    bound_vel = config.is_param_readable(max_finishing_vel, "max_finishing_vel");

}

DepthRegulConfig::~DepthRegulConfig()
{

}

DepthRegulator::DepthRegulator(CmdFixDepth msg, shared_ptr<const DepthRegulConfig> config):
    Regulator(msg.id, {Axis::TZ}, msg.timeout),
    controller(config->kp, config->ki, config->kd),
    cmd_depth(msg.value),
    coord_system(static_cast<CoordSystem>(msg.coord_system)),
    config(config)
{

}

DepthRegulator::DepthRegulator(CmdFixDepthConf msg, shared_ptr<const DepthRegulConfig> config):
    Regulator(msg.id, {Axis::TZ}, msg.timeout),
    controller(msg.kp, msg.ki, msg.kd),
    cmd_depth(msg.value),
    coord_system(static_cast<CoordSystem>(msg.coord_system)),
    config(config)
{

}

DepthRegulator::~DepthRegulator()
{

}

void DepthRegulator::initialize(const NavigInfo& msg)
{
    if (coord_system == CoordSystem::ABS) {
        target_depth = cmd_depth;
    } else {
        target_depth = cmd_depth + msg.depth;
    }
    set_log_headers({"dpth_w", "dpth_r"});
}

void DepthRegulator::update(const NavigInfo& msg)
{
    double err = target_depth - msg.depth;
    double err_d = -msg.velocity_depth;
    double thrust = controller.update(err, err_d) + config->buoyancy_thrust;
    if (config->bound_vel) {
        set_success(fabs(err) < config->accuracy && fabs(msg.velocity_depth) < config->max_finishing_vel);
    } else {
        set_success(fabs(err) < config->accuracy);
    }
    set_thrusts({{Axis::TZ, thrust}});
    set_log_values({target_depth, msg.depth});
}

REG_REGUL(depth, DepthRegulator, CmdFixDepth, DepthRegulConfig);
REG_REGUL(depth, DepthRegulator, CmdFixDepthConf, DepthRegulConfig);
