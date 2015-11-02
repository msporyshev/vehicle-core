#include "motion_client.h"

#include <msg/motion/commands/MsgFixThrust.h>

#include <msg/motion/commands/MsgFixHeading.h>
#include <msg/motion/commands/MsgFixHeadingConf.h>

#include <msg/motion/commands/MsgFixDepth.h>
#include <msg/motion/commands/MsgFixDepthConf.h>

#include <msg/motion/commands/MsgFixPitch.h>
#include <msg/motion/commands/MsgFixPitchConf.h>

#include <msg/motion/commands/MsgFixPosition.h>
#include <msg/motion/commands/MsgFixPositionConf.h>

#include <msg/motion/commands/MsgFixVelocity.h>
#include <msg/motion/commands/MsgFixVert.h>

#include <ipc_lib.h>

#include <iostream>

using namespace Central;

static const bool NAVIG_COMPATIBLE_MODE = true;

MotionClient::MotionClient()
{
    register_outcoming_message<MsgFixThrust>();
    register_outcoming_message<MsgFixHeading>();
    register_outcoming_message<MsgFixHeadingConf>();
    register_outcoming_message<MsgFixDepth>();
    register_outcoming_message<MsgFixDepthConf>();
    register_outcoming_message<MsgFixPitch>();
    register_outcoming_message<MsgFixPitchConf>();
    register_outcoming_message<MsgFixPosition>();
    register_outcoming_message<MsgFixPositionConf>();
    register_outcoming_message<MsgFixVelocity>();
    register_outcoming_message<MsgFixVert>();

    subscribe(*this, handle_msg_cmd_status);
}

MotionClient::~MotionClient()
{

}

void MotionClient::handle_msg_cmd_status(MsgCmdStatus msg)
{
    cmd_history[msg.id] = static_cast<CmdStatus>(msg.status);
}

CmdStatus MotionClient::wait_for(int id)
{
    while (cmd_history.find(id) == cmd_history.end()) {
        process_messages(0);
    }
    return cmd_history[id];
}

int MotionClient::generate_cmd_id()
{
    return last_cmd_id++;
}


void MotionClient::thrust(Axis axis, double value, double timeout, WaitMode wm)
{
    MsgFixThrust msg;
    msg.axis = static_cast<int>(axis);
    msg.value = value;
    publish_cmd(msg, timeout, wm);
}

void MotionClient::unfix_all()
{
    // отключение всех стабилизаций равносильно активации тяги по всем осям с нулевым таймаутом
    thrust(Axis::TX, 0.0, 0.0);
    thrust(Axis::TY, 0.0, 0.0);
    thrust(Axis::TZ, 0.0, 0.0);
    thrust(Axis::MX, 0.0, 0.0);
    thrust(Axis::MY, 0.0, 0.0);
    thrust(Axis::MZ, 0.0, 0.0);
}

void MotionClient::fix_heading(double value, CoordSystem coord_system, double timeout, WaitMode wm)
{
    MsgFixHeading msg;
    msg.value = value;
    msg.coord_system = static_cast<int>(coord_system);
    publish_cmd(msg, timeout, wm);
}

void MotionClient::fix_heading(double value, double timeout, WaitMode wm)
{
    fix_heading(value, CoordSystem::ABS, timeout, wm);
}

void MotionClient::fix_heading(double value, double timeout, double kp, double ki, double kd, WaitMode wm)
{
    MsgFixHeadingConf msg;
    msg.value = value;
    msg.coord_system = static_cast<int>(CoordSystem::ABS);
    msg.kp = kp;
    msg.ki = ki;
    msg.kd = kd;
    publish_cmd(msg, timeout, wm);
}

void MotionClient::turn_right(double value, double timeout, WaitMode wm)
{
    fix_heading(value, CoordSystem::REL, timeout, wm);
}

void MotionClient::turn_left(double value, double timeout, WaitMode wm)
{
    fix_heading(-value, CoordSystem::REL, timeout, wm);
}

void MotionClient::fix_depth(double value, CoordSystem coord_system, double timeout, WaitMode wm)
{
    MsgFixDepth msg;
    msg.value = value;
    msg.coord_system = static_cast<int>(coord_system);
    publish_cmd(msg, timeout, wm);
}

void MotionClient::fix_depth(double value, double timeout, double kp, double ki, double kd, WaitMode wm)
{
    MsgFixDepthConf msg;
    msg.value = value;
    msg.coord_system = static_cast<int>(CoordSystem::ABS);
    msg.kp = kp;
    msg.ki = ki;
    msg.kd = kd;
    publish_cmd(msg, timeout, wm);
}

void MotionClient::fix_depth(double value, double timeout, WaitMode wm)
{
    fix_depth(value, CoordSystem::ABS, timeout, wm);
}

void MotionClient::move_down(double value, double timeout, WaitMode wm)
{
    fix_depth(value, CoordSystem::REL, timeout, wm);
}

void MotionClient::move_up(double value, double timeout, WaitMode wm)
{
    fix_depth(-value, CoordSystem::REL, timeout, wm);
}

void MotionClient::fix_pitch(double value, CoordSystem coord_system, double timeout, WaitMode wm)
{
    MsgFixPitch msg;
    msg.value = value;
    msg.coord_system = static_cast<int>(coord_system);
    publish_cmd(msg, timeout, wm);
}

void MotionClient::fix_pitch(double value, double timeout, WaitMode wm)
{
    fix_pitch(value, CoordSystem::ABS, timeout, wm);
}

void MotionClient::fix_pitch(double value, double timeout, double kp, double ki, double kd, WaitMode wm)
{
    MsgFixPitchConf msg;
    msg.value = value;
    msg.coord_system = static_cast<int>(CoordSystem::ABS);
    msg.kp = kp;
    msg.ki = ki;
    msg.kd = kd;
    publish_cmd(msg, timeout, wm);
}

void MotionClient::turn_up(double value, double timeout, WaitMode wm)
{
    fix_pitch(value, CoordSystem::REL, timeout, wm);
}

void MotionClient::turn_down(double value, double timeout, WaitMode wm)
{
    fix_pitch(-value, CoordSystem::REL, timeout, wm);
}

void MotionClient::fix_position(Point2d value, MoveMode move_mode, CoordSystem coord_system, double timeout,
    WaitMode wm)
{
    if (NAVIG_COMPATIBLE_MODE) {
        value = MakePoint2(value.y, value.x);
    }

    MsgFixPosition msg;
    msg.x = value.x;
    msg.y = value.y;
    msg.move_mode = static_cast<int>(move_mode);
    msg.coord_system = static_cast<int>(coord_system);
    publish_cmd(msg, timeout, wm);
}

void MotionClient::fix_position(Point2d value, MoveMode move_mode, double timeout, WaitMode wm)
{
    fix_position(value, move_mode, CoordSystem::ABS, timeout, wm);
}

void MotionClient::unseat(Point2d value, MoveMode move_mode, double timeout, WaitMode wm)
{
    fix_position(value, move_mode, CoordSystem::REL, timeout, wm);
}

void MotionClient::fix_position(Point2d value, MoveMode move_mode, double timeout,
        double fwd_kp, double fwd_ki, double fwd_kd, double side_kp, double side_ki, double side_kd,
        WaitMode wm)
{
    if (NAVIG_COMPATIBLE_MODE) {
        value = MakePoint2(value.y, value.x);
    }

    MsgFixPositionConf msg;
    msg.x = value.x;
    msg.y = value.y;
    msg.move_mode = static_cast<int>(move_mode);
    msg.coord_system = static_cast<int>(CoordSystem::ABS);
    msg.fwd_kp = fwd_kp;
    msg.fwd_ki = fwd_ki;
    msg.fwd_kd = fwd_kd;

    msg.side_kp = side_kp;
    msg.side_ki = side_ki;
    msg.side_kd = side_kd;
    publish_cmd(msg, timeout, wm);
}

void MotionClient::move_forward(double value, MoveMode move_mode, double timeout, WaitMode wm)
{
    auto target_pos = MakePoint2(value, 0.0);
    if (NAVIG_COMPATIBLE_MODE) {
        target_pos = MakePoint2(target_pos.y, target_pos.x);
    }
    fix_position(target_pos, move_mode, CoordSystem::REL, timeout, wm);
}

void MotionClient::move_backward(double value, MoveMode move_mode, double timeout, WaitMode wm)
{
    auto target_pos = MakePoint2(-value, 0.0);
    if (NAVIG_COMPATIBLE_MODE) {
        target_pos = MakePoint2(target_pos.y, target_pos.x);
    }
    fix_position(target_pos, move_mode, CoordSystem::REL, timeout, wm);
}

void MotionClient::move_right(double value, MoveMode move_mode, double timeout, WaitMode wm)
{
    auto target_pos = MakePoint2(0.0, value);
    if (NAVIG_COMPATIBLE_MODE) {
        target_pos = MakePoint2(target_pos.y, target_pos.x);
    }
    fix_position(target_pos, move_mode, CoordSystem::REL, timeout, wm);
}

void MotionClient::move_left(double value, MoveMode move_mode, double timeout, WaitMode wm)
{
    auto target_pos = MakePoint2(0.0, -value);
    if (NAVIG_COMPATIBLE_MODE) {
        target_pos = MakePoint2(target_pos.y, target_pos.x);
    }
    fix_position(target_pos, move_mode, CoordSystem::REL, timeout, wm);
}

void MotionClient::fix_velocity(double value, double timeout, WaitMode wm)
{
    MsgFixVelocity msg;
    msg.value = value;
    publish_cmd(msg, timeout, wm);
}

void MotionClient::fix_vert(double value, SpeedyVertMode mode, double timeout, WaitMode wm)
{
    MsgFixVert msg;
    msg.value = value;
    msg.mode = static_cast<int>(mode);
    publish_cmd(msg, timeout, wm);
}
