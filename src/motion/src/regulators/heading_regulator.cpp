#include "regulators/heading_regulator.h"

#include "registry.h"
#include <libauv/utils/math_u.h>

using namespace std;
using namespace utils;

using motion::CmdFixHeading;
using motion::CmdFixHeadingConf;

HeadingRegulConfig::HeadingRegulConfig(const YamlReader& config)
{
    config.SET_PARAM(kp);
    config.SET_PARAM(ki);
    config.SET_PARAM(kd);

    config.SET_PARAM(accuracy);
    bound_vel = config.is_param_readable(max_finishing_vel, "max_finishing_vel");
}

HeadingRegulConfig::~HeadingRegulConfig()
{

}

HeadingRegulator::HeadingRegulator(CmdFixHeading msg, shared_ptr<const HeadingRegulConfig> config):
    Regulator(msg.id, {Axis::MZ}, msg.timeout),
    controller(config->kp, config->ki, config->kd),
    cmd_heading(normalize_angle(msg.value)),
    coord_system(static_cast<CoordSystem>(msg.coord_system)),
    config(config)
{

}

HeadingRegulator::HeadingRegulator(CmdFixHeadingConf msg, shared_ptr<const HeadingRegulConfig> config):
    Regulator(msg.id, {Axis::MZ}, msg.timeout),
    controller(msg.kp, msg.ki, msg.kd),
    cmd_heading(normalize_angle(msg.value)),
    coord_system(static_cast<CoordSystem>(msg.coord_system)),
    config(config)
{

}

HeadingRegulator::~HeadingRegulator()
{

}

void HeadingRegulator::initialize(const NavigInfo& msg)
{
    if (coord_system == CoordSystem::ABS) {
        target_heading = cmd_heading;
    } else {
        target_heading = normalize_angle(cmd_heading + msg.heading);
    }
    set_log_headers({"hd_w", "hd_r"});
}

void HeadingRegulator::update(const NavigInfo& msg)
{
    double heading = normalize_angle(msg.heading);
    double err = normalize_angle(target_heading - heading);
    double err_d = -msg.heading_rate;
    double thrust = controller.update(err, err_d);
    if (config->bound_vel) {
        set_success(fabs(err) < config->accuracy && fabs(msg.heading_rate) < config->max_finishing_vel);
    } else {
        set_success(fabs(err) < config->accuracy);
    }
    set_thrusts({{Axis::MZ, thrust}});
    set_log_values({target_heading, heading});
}

REG_REGUL(heading, HeadingRegulator, CmdFixHeading, HeadingRegulConfig);
REG_REGUL(heading, HeadingRegulator, CmdFixHeadingConf, HeadingRegulConfig);
