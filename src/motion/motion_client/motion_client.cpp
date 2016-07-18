#include "motion_client.h"

#include <ros/message_traits.h>
#include <ros/ros.h>

#include <motion/CmdFixDepth.h>
#include <motion/CmdFixHeading.h>
#include <motion/CmdFixPitch.h>
#include <motion/CmdFixPosition.h>
#include <motion/CmdFixThrust.h>
#include <motion/CmdFixVelocity.h>
#include <motion/CmdFixVert.h>
#include <motion/CmdFixHeight.h>
#include <motion/CmdFixTarget.h>
#include <motion/CmdFixTargetDistance.h>

#include <libauv/utils/math_u.h>

#include <iostream>

using ros::message_traits::datatype;
using namespace motion;


int MotionClient::last_cmd_id = 0;

MotionClient::MotionClient(ipc::Communicator& com) :
    communicator_(com)
{
    publishers_[datatype(CmdFixThrust())] = communicator_.advertise_cmd<CmdFixThrust>("motion");
    publishers_[datatype(CmdFixHeading())] = communicator_.advertise_cmd<CmdFixHeading>("motion");
    publishers_[datatype(CmdFixDepth())] = communicator_.advertise_cmd<CmdFixDepth>("motion");
    publishers_[datatype(CmdFixPitch())] = communicator_.advertise_cmd<CmdFixPitch>("motion");
    publishers_[datatype(CmdFixPosition())] = communicator_.advertise_cmd<CmdFixPosition>("motion");
    publishers_[datatype(CmdFixVelocity())] = communicator_.advertise_cmd<CmdFixVelocity>("motion");
    publishers_[datatype(CmdFixVert())] = communicator_.advertise_cmd<CmdFixVert>("motion");
    publishers_[datatype(CmdFixHeight())] = communicator_.advertise_cmd<CmdFixHeight>("motion");
    publishers_[datatype(CmdFixTarget())] = communicator_.advertise_cmd<CmdFixTarget>("motion");
    publishers_[datatype(CmdFixTargetDistance())] = communicator_.advertise_cmd<CmdFixTargetDistance>("motion");

    cmd_sub_ = communicator_.subscribe("motion", &MotionClient::handle_msg_cmd_status, this, 10);
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
    while (cmd_history.count(id) == 0 && loop.ok()) {
    }
    if (cmd_history.count(id) == 0) {
        ROS_INFO_STREAM("Didn't wait for command!!! " << id);
        ROS_INFO_STREAM("Loop ok: " << (loop.ok() ? "true" : "false"));
    }

    if (!ros::ok()) {
        ROS_INFO("Ros communication failed");
        unfix_all();
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
    motion::CmdFixHeading msg;
    msg.value = value;
    msg.coord_system = static_cast<int>(coord_system);
    publish_cmd(msg, timeout, wm);
}

void MotionClient::fix_heading(double value, double timeout, WaitMode wm)
{
    fix_heading(value, CoordSystem::ABS, timeout, wm);
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
    motion::CmdFixPitch msg;
    msg.value = value;
    msg.coord_system = static_cast<int>(coord_system);
    publish_cmd(msg, timeout, wm);
}

void MotionClient::fix_pitch(double value, double timeout, WaitMode wm)
{
    fix_pitch(value, CoordSystem::ABS, timeout, wm);
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

    motion::CmdFixPosition msg;
    msg.x = value.x;
    msg.y = value.y;
    msg.move_mode = static_cast<int>(move_mode);
    msg.coord_system = static_cast<int>(coord_system);
    publish_cmd(msg, timeout, wm);
}

void MotionClient::fix_target(Point2d value, double timeout, WaitMode wm)
{
    motion::CmdFixTargetDistance msg;
    msg.x = value.x;
    msg.y = value.y;
    msg.distance = -1;
    msg.coord_system = static_cast<int>(CoordSystem::ABS);
    publish_cmd(msg, timeout, wm);
}

void MotionClient::fix_target(Point2d value, double distance, double timeout, WaitMode wm)
{
    motion::CmdFixTargetDistance msg;
    msg.x = value.x;
    msg.y = value.y;
    msg.distance = distance;
    msg.coord_system = static_cast<int>(CoordSystem::ABS);
    publish_cmd(msg, timeout, wm);
}

void MotionClient::fix_target(double distance, double timeout, WaitMode wm)
{
    motion::CmdFixTargetDistance msg;
    msg.distance = distance;
    msg.coord_system = static_cast<int>(CoordSystem::REL);
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

void MotionClient::move_forward(double value, MoveMode move_mode, double timeout, WaitMode wm)
{
    auto target_pos = Point2d(value, 0.0);
    fix_position(target_pos, move_mode, CoordSystem::REL, timeout, wm);
}

void MotionClient::move_backward(double value, MoveMode move_mode, double timeout, WaitMode wm)
{
    auto target_pos = Point2d(-value, 0.0);
    fix_position(target_pos, move_mode, CoordSystem::REL, timeout, wm);
}

void MotionClient::move_right(double value, MoveMode move_mode, double timeout, WaitMode wm)
{
    auto target_pos = Point2d(0.0, value);
    fix_position(target_pos, move_mode, CoordSystem::REL, timeout, wm);
}

void MotionClient::move_left(double value, MoveMode move_mode, double timeout, WaitMode wm)
{
    auto target_pos = Point2d(0.0, -value);
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
