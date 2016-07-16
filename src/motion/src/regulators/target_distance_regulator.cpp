#include "regulators/target_distance_regulator.h"

#include "registry.h"

#include <utils/math_u.h>

using namespace std;
using namespace utils;

using motion::CmdFixTargetDistance;

TargetDistanceRegulConfig::TargetDistanceRegulConfig(const YamlReader& config): PidRegulConfig(config)
{
    config.SET_PARAM(accuracy);
    bound_vel = config.is_param_readable(max_finishing_vel, "max_finishing_vel");

    YAML::Node dependencies = config.read_as<YAML::Node>("dependencies");
    target_config = make_shared<TargetRegulConfig>(YamlReader(dependencies["target"]));
}

TargetDistanceRegulator::TargetDistanceRegulator(CmdFixTargetDistance msg,
    std::shared_ptr<const TargetDistanceRegulConfig> config)
        : Regulator(msg.id, {Axis::TX, Axis::MZ}, msg.timeout)
        , controller(config->kp, config->ki, config->kd)
        , coord_system((CoordSystem)msg.coord_system)
        , cmd_position(Point2d(msg.x, msg.y))
        , cmd_distance(msg.distance)
        , config(config)
{

}

void TargetDistanceRegulator::initialize(const NavigInfo& msg)
{
    if (coord_system == CoordSystem::ABS) {
        target_position = cmd_position;
    } else {
        target_position = msg.position +
            Point2d(cmd_distance * cos(utils::to_rad(msg.heading)),
            cmd_distance * sin(utils::to_rad(msg.heading)));
    }

    if (cmd_distance < 0) {
        cmd_distance = norm(msg.position - target_position);
    }

    motion::CmdFixTarget fix_target_cmd;
    fix_target_cmd.x = target_position.x;
    fix_target_cmd.y = target_position.y;
    LOG << "initializing target regulator" << endl;
    LOG << "msg.x: " << fix_target_cmd.x << " msg.y: " << fix_target_cmd.y << endl;
    LOG << "msg.distance: " << cmd_distance << endl;
    fix_target_cmd.id = -1;
    fix_target_cmd.timeout = 0.0;

    target_regulator = make_shared<TargetRegulator>(fix_target_cmd, config->target_config);
    target_regulator->activate();
    target_regulator->handle_navig(msg);

    add_dependency(target_regulator);

    set_log_headers({"dist_w", "dist_r"});
}

void TargetDistanceRegulator::update(const NavigInfo& msg)
{
    auto current_position = msg.position;
    auto current_distance = norm(target_position - current_position);

    auto target_heading = kurs_point1_to_point2(current_position.x, current_position.y,
        cmd_position.x, cmd_position.y);
    auto heading_delta = degree_angle_diff(target_heading, msg.heading);
    double delta_rad = to_rad(heading_delta);
    int sign = cos(delta_rad) > 0 ? 1 : -1;
    double d = current_distance;
    double r = cmd_distance;
    double discr = r * r - d * d * sin(delta_rad) * sin(delta_rad);

    if (discr < 0) {
        LOG << "Wait for heading" << endl;
        set_thrusts({{Axis::TX, 0}});
        set_log_values({cmd_distance, current_distance});
        return;
    }

    LOG << "heading delta: " << heading_delta << endl;
    LOG << "current_distance * cos: " << current_distance * cos(delta_rad) << endl;
    LOG << "sign: " << sign << endl;
    LOG << "p: " << sqrt(discr) << endl;
    double err = current_distance * cos(delta_rad) - sign * sqrt(discr);
    double err_d = -msg.velocity_forward;

    auto thrust_fwd = controller.update(err, err_d);

    if (config->bound_vel) {
        LOG << "bound_vel" << endl;
        set_success(norm(err), config->accuracy, norm(err_d), config->max_finishing_vel);
    } else {
        LOG << "err: " << err << endl;
        set_success(norm(err), config->accuracy);
    }

    // if (target_regulator->has_succeeded()) {
        set_thrusts({{Axis::TX, thrust_fwd}});
    // } else {
        // LOG << "waiting for heading stab" << endl;
        // set_thrusts({{Axis::TX, 0.0}});
    // }

    set_log_values({cmd_distance, current_distance});
}

REG_REGUL(target_distance, TargetDistanceRegulator, CmdFixTargetDistance, TargetDistanceRegulConfig);
