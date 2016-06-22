#include "motion_client.h"

#include <ros/message_traits.h>

#include <motion/CmdFixDepth.h>
#include <motion/CmdFixDepthConf.h>
#include <motion/CmdFixHeading.h>
#include <motion/CmdFixHeadingConf.h>
#include <motion/CmdFixPitch.h>
#include <motion/CmdFixPitchConf.h>
#include <motion/CmdFixPosition.h>
#include <motion/CmdFixPositionConf.h>
#include <motion/CmdFixThrust.h>
#include <motion/CmdFixVelocity.h>
#include <motion/CmdFixVert.h>

#include <libauv/utils/math_u.h>

#include <iostream>

using ros::message_traits::datatype;
using namespace motion;

static const bool NAVIG_COMPATIBLE_MODE = true;

int MotionClient::last_cmd_id = 0;

MotionClient::MotionClient(ipc::Communicator& com) :
    communicator_(com)
{
    publishers_[datatype(CmdFixThrust())] = communicator_.advertise_cmd<CmdFixThrust>("motion");
    publishers_[datatype(CmdFixHeading())] = communicator_.advertise_cmd<CmdFixHeading>("motion");
    publishers_[datatype(CmdFixHeadingConf())] = communicator_.advertise_cmd<CmdFixHeadingConf>("motion");
    publishers_[datatype(CmdFixDepth())] = communicator_.advertise_cmd<CmdFixDepth>("motion");
    publishers_[datatype(CmdFixDepthConf())] = communicator_.advertise_cmd<CmdFixDepthConf>("motion");
    publishers_[datatype(CmdFixPitch())] = communicator_.advertise_cmd<CmdFixPitch>("motion");
    publishers_[datatype(CmdFixPitchConf())] = communicator_.advertise_cmd<CmdFixPitchConf>("motion");
    publishers_[datatype(CmdFixPosition())] = communicator_.advertise_cmd<CmdFixPosition>("motion");
    publishers_[datatype(CmdFixPositionConf())] = communicator_.advertise_cmd<CmdFixPositionConf>("motion");
    publishers_[datatype(CmdFixVelocity())] = communicator_.advertise_cmd<CmdFixVelocity>("motion");
    publishers_[datatype(CmdFixVert())] = communicator_.advertise_cmd<CmdFixVert>("motion");

    cmd_sub_ = communicator_.subscribe("motion", &MotionClient::handle_msg_cmd_status, this);
}

MotionClient::~MotionClient()
{

}

void MotionClient::handle_msg_cmd_status(const motion::MsgCmdStatus& msg)
{
    cmd_history[msg.id] = static_cast<CmdStatus>(msg.status);
}

CmdStatus MotionClient::wait_for(int id)
{
    ipc::EventLoop loop(10);
    // этот цикл выходил слишком рано, из за loop.ok(), однако воспроизвести пока не получилось
    while (cmd_history.find(id) == cmd_history.end() && loop.ok()) {
    }
    return cmd_history[id];
}

int MotionClient::generate_cmd_id()
{
    return last_cmd_id++;
}


void MotionClient::thrust(Axis axis, double value, double timeout, WaitMode wm)
{
    motion::CmdFixThrust msg;
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
    value *= DEG_to_R_;
    motion::CmdFixHeading msg;
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
    motion::CmdFixHeadingConf msg;
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
    motion::CmdFixDepth msg;
    msg.value = value;
    msg.coord_system = static_cast<int>(coord_system);
    publish_cmd(msg, timeout, wm);
}

void MotionClient::fix_depth(double value, double timeout, double kp, double ki, double kd, WaitMode wm)
{
    motion::CmdFixDepthConf msg;
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
    value *= DEG_to_R_;
    motion::CmdFixPitch msg;
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
    motion::CmdFixPitchConf msg;
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

void MotionClient::fix_position(libauv::Point2d value, MoveMode move_mode, CoordSystem coord_system, double timeout,
    WaitMode wm)
{
    if (NAVIG_COMPATIBLE_MODE) {
        value = MakePoint2(value.y, value.x);
    }

    motion::CmdFixPosition msg;
    msg.x = value.x;
    msg.y = value.y;
    msg.move_mode = static_cast<int>(move_mode);
    msg.coord_system = static_cast<int>(coord_system);
    publish_cmd(msg, timeout, wm);
}

void MotionClient::fix_position(libauv::Point2d value, MoveMode move_mode, double timeout, WaitMode wm)
{
    fix_position(value, move_mode, CoordSystem::ABS, timeout, wm);
}

void MotionClient::unseat(libauv::Point2d value, MoveMode move_mode, double timeout, WaitMode wm)
{
    fix_position(value, move_mode, CoordSystem::REL, timeout, wm);
}

void MotionClient::fix_position(libauv::Point2d value, MoveMode move_mode, double timeout,
        double fwd_kp, double fwd_ki, double fwd_kd, double side_kp, double side_ki, double side_kd,
        WaitMode wm)
{
    if (NAVIG_COMPATIBLE_MODE) {
        value = MakePoint2(value.y, value.x);
    }

    motion::CmdFixPositionConf msg;
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
    motion::CmdFixVelocity msg;
    msg.value = value;
    publish_cmd(msg, timeout, wm);
}

void MotionClient::fix_vert(double value, SpeedyVertMode mode, double timeout, WaitMode wm)
{
    motion::CmdFixVert msg;
    msg.value = value;
    msg.mode = static_cast<int>(mode);
    publish_cmd(msg, timeout, wm);
}
