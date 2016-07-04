#include "regulators/position_regulator.h"

#include "registry.h"

#include <utils/math_u.h>

using namespace std;
using namespace utils;

using motion::CmdFixPosition;
using motion::CmdFixPositionConf;

PositionRegulConfig::PositionRegulConfig(const YamlReader& config)
{

    config.SET_PARAM(cruiser_dist);

    config.SET_PARAM(fwd_kp);
    config.SET_PARAM(fwd_ki);
    config.SET_PARAM(fwd_kd);

    config.SET_PARAM(side_kp);
    config.SET_PARAM(side_ki);
    config.SET_PARAM(side_kd);

    config.SET_PARAM(accuracy);
    bound_vel = config.is_param_readable(max_finishing_vel, "max_finishing_vel");

    YAML::Node dependencies = config.read_as<YAML::Node>("dependencies");
    heading_config = make_shared<HeadingRegulConfig>(YamlReader(dependencies["heading"]));
}

PositionRegulator::PositionRegulator(CmdFixPosition msg, std::shared_ptr<const PositionRegulConfig> config):
    Regulator(msg.id, {Axis::TX, Axis::TY, Axis::MZ}, msg.timeout),
    fwd_controller(config->fwd_kp, config->fwd_ki, config->fwd_kd),
    side_controller(config->side_kp, config->side_ki, config->side_kd),
    cmd_position(MakePoint2(msg.x, msg.y)),
    mode(static_cast<MoveMode>(msg.move_mode)),
    coord_system(static_cast<CoordSystem>(msg.coord_system)),
    config(config)
{

}

PositionRegulator::PositionRegulator(CmdFixPositionConf msg, std::shared_ptr<const PositionRegulConfig> config):
    Regulator(msg.id, {Axis::TX, Axis::TY, Axis::MZ}, msg.timeout),
    fwd_controller(msg.fwd_kp, msg.fwd_ki, msg.fwd_kd),
    side_controller(msg.side_kp, msg.side_ki, msg.side_kd),
    cmd_position(MakePoint2(msg.x, msg.y)),
    mode(static_cast<MoveMode>(msg.move_mode)),
    coord_system(static_cast<CoordSystem>(msg.coord_system)),
    config(config)
{
    LOG << "handle_msg mode: " << msg.move_mode << endl;
}

PositionRegulator::~PositionRegulator()
{

}

libauv::Point2d PositionRegulator::get_current_position(const NavigInfo& msg)
{
    return msg.position;
}

void PositionRegulator::initialize(const NavigInfo& msg)
{
    if (coord_system == CoordSystem::ABS) {
        target_position = cmd_position;
    } else {
        double x = cmd_position.x;
        double y = cmd_position.y;
        libauv::Point2d delta = MakePoint2((x * cos(msg.heading) - y * sin(msg.heading)),
            (x * sin(msg.heading) + y * cos(msg.heading)));
        target_position = get_current_position(msg) + delta;
    }

    if (mode == MoveMode::HEADING_FREE) {
        LOG << "mode == HEADING_FREE" << endl;
    }
    auto current_position = get_current_position(msg);
    if (mode != MoveMode::HEADING_FREE) {
        double target_heading = 0.0;
        if (mode == MoveMode::AUTO) {
            LOG << "mode == AUTO" << endl;
            if (norm(target_position - get_current_position(msg)) < config->cruiser_dist) {
                mode = MoveMode::HOVER;
            } else {
                mode = MoveMode::CRUISE;
            }
        }
        switch (mode) {
            case MoveMode::HOVER:
                LOG << "mode == HOVER" << endl;
                target_heading = msg.heading;
                break;
            case MoveMode::CRUISE:
                LOG << "mode == CRUISE" << endl;
                target_heading = kurs_point1_to_point2(current_position.x, current_position.y,
                target_position.x, target_position.y);
                break;
            default:
                break;
        }

        motion::CmdFixHeading fix_heading_msg;
        fix_heading_msg.value = target_heading;
        fix_heading_msg.coord_system = static_cast<short>(CoordSystem::ABS);
        LOG << "initializing position regulator" << endl;
        LOG << "msg.heading: " << msg.heading << endl;
        LOG << "fix_heading_msg.value: " << fix_heading_msg.value << endl;
        LOG << "target_heading: " << target_heading << endl;
        fix_heading_msg.id = -1;
        fix_heading_msg.timeout = 0.0;
        heading_regulator = make_shared<HeadingRegulator>(fix_heading_msg, config->heading_config);
        heading_regulator->activate();
        heading_regulator->handle_navig(msg);

        add_dependency(heading_regulator);
    } else {
        heading_regulator = nullptr;
    }

    set_log_headers({"x_w", "y_w", "x_r", "y_r"});
}

void PositionRegulator::update(const NavigInfo& msg)
{
    libauv::Point2d current_position = get_current_position(msg);

    double velocity_north = msg.velocity_forward * cos(to_rad(msg.heading)) - msg.velocity_right * sin(to_rad(msg.heading));
    double velocity_east = msg.velocity_forward * sin(to_rad(msg.heading)) + msg.velocity_right * cos(to_rad(msg.heading));

    auto err = target_position - current_position;
    double x = err.x;
    double y = err.y;
    err = MakePoint2((x * cos(msg.heading) + y * sin(msg.heading)),
        (- x * sin(msg.heading) + y * cos(msg.heading)));

    auto err_d = MakePoint2(-velocity_north, -velocity_east);
    x = err_d.x;
    y = err_d.y;
    err_d = MakePoint2(x * cos(msg.heading) + y * sin(msg.heading), - x * sin(msg.heading) + y * cos(msg.heading));

    auto thrust_fwd = fwd_controller.update(err.x, err_d.x);
    auto thrust_side = side_controller.update(err.y, err_d.y);

    if (config->bound_vel) {
        LOG << "bound_vel" << endl;
        set_success(norm(err), config->accuracy, norm(err_d), config->max_finishing_vel);
    } else {
        LOG << "err_x: " << err.x << ", err_y: " << err.y << endl;
        set_success(norm(err), config->accuracy);
    }

    if (mode == MoveMode::HEADING_FREE) {
        set_thrusts({{Axis::TX, thrust_fwd}, {Axis::TY, thrust_side}});
    } else if (heading_regulator->has_succeeded()) {
        set_thrusts({{Axis::TX, thrust_fwd}, {Axis::TY, thrust_side}});
    } else {
        LOG << "waiting for heading stab" << endl;
        set_thrusts({{Axis::TX, 0.0}, {Axis::TY, 0.0}});
    }

    set_log_values({target_position.x, target_position.y, current_position.x, current_position.y});
}

REG_REGUL(position, PositionRegulator, CmdFixPosition, PositionRegulConfig);
REG_REGUL(position, PositionRegulator, CmdFixPositionConf, PositionRegulConfig);
