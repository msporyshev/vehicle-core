#include "regulators/target_regulator.h"

#include "registry.h"

#include <utils/math_u.h>

using namespace std;
using namespace utils;

using motion::CmdFixTarget;

TargetRegulConfig::TargetRegulConfig(const YamlReader& config): PidRegulConfig(config)
{
    config.SET_PARAM(accuracy);
    bound_vel = config.is_param_readable(max_finishing_vel, "max_finishing_vel");
}

TargetRegulator::TargetRegulator(CmdFixTarget msg, std::shared_ptr<const TargetRegulConfig> config):
    Regulator(msg.id, {Axis::MZ}, msg.timeout),
    controller(config->kp, config->ki, config->kd),
    cmd_position(Point2d(msg.x, msg.y)),
    config(config)
{

}

Point2d TargetRegulator::get_current_position(const NavigInfo& msg)
{
    return msg.position;
}

void TargetRegulator::initialize(const NavigInfo& msg)
{
    auto current_position = get_current_position(msg);

    set_log_headers({"h_w", "h_r"});
}

void TargetRegulator::update(const NavigInfo& msg)
{
    Point2d current_position = get_current_position(msg);

    target_heading = kurs_point1_to_point2(current_position.x, current_position.y,
        cmd_position.x, cmd_position.y);

    double heading = normalize_degree_angle(msg.heading);
    double err = normalize_degree_angle(target_heading - heading);
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

REG_REGUL(target, TargetRegulator, CmdFixTarget, TargetRegulConfig);
