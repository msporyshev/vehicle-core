#include "robosub_conf_motion_client.h"

using namespace std;

void RobosubConfMotionClient::thrust_forward(double value, double timeout, WaitMode wm)
{
    thrust(Axis::TX, value, timeout, wm);
}

void RobosubConfMotionClient::thrust_backward(double value, double timeout, WaitMode wm)
{
    thrust(Axis::TX, -value, timeout, wm);
}

void RobosubConfMotionClient::thrust_right(double value, double timeout, WaitMode wm)
{
    thrust(Axis::TY, value, timeout, wm);
}

void RobosubConfMotionClient::thrust_left(double value, double timeout, WaitMode wm)
{
    thrust(Axis::TY, -value, timeout, wm);
}

void RobosubConfMotionClient::thrust_down(double value, double timeout, WaitMode wm)
{
    thrust(Axis::TZ, value, timeout, wm);
}

void RobosubConfMotionClient::thrust_up(double value, double timeout, WaitMode wm)
{
    thrust(Axis::TZ, -value, timeout, wm);
}

void RobosubConfMotionClient::thrust_pitch(double value, double timeout, WaitMode wm)
{
    thrust(Axis::MY, value, timeout, wm);
}

void RobosubConfMotionClient::thrust_heading(double value, double timeout, WaitMode wm)
{
    thrust(Axis::MZ, value, timeout, wm);
}

void RobosubConfMotionClient::unfix_all()
{
    MotionClient::unfix_all();
}

void RobosubConfMotionClient::fix_heading(double value, double timeout, WaitMode wm)
{
    MotionClient::fix_heading(value, timeout, wm);
}

void RobosubConfMotionClient::fix_heading(double value, double timeout, double kp, double ki, double kd, WaitMode wm)
{
    MotionClient::fix_heading(value, timeout, kp, ki, kd, wm);
}


void RobosubConfMotionClient::turn_right(double bearing, double timeout, WaitMode wm)
{
    MotionClient::turn_right(bearing, timeout, wm);
}

void RobosubConfMotionClient::turn_left(double bearing, double timeout, WaitMode wm)
{
    MotionClient::turn_left(bearing, timeout, wm);
}


void RobosubConfMotionClient::fix_pitch(double value, double timeout, WaitMode wm)
{
    MotionClient::fix_pitch(value, timeout, wm);
}

void RobosubConfMotionClient::fix_pitch(double value, double timeout, double kp, double ki, double kd, WaitMode wm)
{
    MotionClient::fix_pitch(value, timeout, kp, ki, kd, wm);
}


void RobosubConfMotionClient::fix_depth(double value, double timeout, WaitMode wm)
{
    MotionClient::fix_depth(value, timeout, wm);
}

void RobosubConfMotionClient::fix_depth(double value, double timeout, double kp, double ki, double kd, WaitMode wm)
{
    MotionClient::fix_depth(value, timeout, kp, ki, kd, wm);
}


void RobosubConfMotionClient::move_down(double depth, double timeout, WaitMode wm)
{
    MotionClient::move_down(depth, timeout, wm);
}

void RobosubConfMotionClient::move_up(double depth, double timeout, WaitMode wm)
{
    MotionClient::move_up(depth, timeout, wm);
}


void RobosubConfMotionClient::fix_position(libauv::Point2d value, MoveMode move_mode, double timeout, WaitMode wm)
{
    MotionClient::fix_position(value, move_mode, timeout, wm);
}

void RobosubConfMotionClient::fix_position(libauv::Point2d value, MoveMode move_mode, double timeout,
    double fwd_kp, double fwd_ki, double fwd_kd, double side_kp, double side_ki, double side_kd,
    WaitMode wm)
{
    MotionClient::fix_position(value, move_mode, timeout, fwd_kp, fwd_ki, fwd_kd, side_kp, side_ki, side_kd, wm);
}


void RobosubConfMotionClient::move_forward(double value, double timeout, WaitMode wm)
{
    MotionClient::move_forward(value, MoveMode::HOVER, timeout, wm);
}

void RobosubConfMotionClient::move_backward(double value, double timeout, WaitMode wm)
{
    MotionClient::move_backward(value, MoveMode::HOVER, timeout, wm);
}

void RobosubConfMotionClient::move_right(double value, double timeout, WaitMode wm)
{
    MotionClient::move_right(value, MoveMode::HOVER, timeout, wm);
}

void RobosubConfMotionClient::move_left(double value, double timeout, WaitMode wm)
{
    MotionClient::move_left(value, MoveMode::HOVER, timeout, wm);
}

