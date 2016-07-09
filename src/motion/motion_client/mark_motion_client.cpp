#include "mark_motion_client.h"
#include "move_mode.h"

using namespace std;

void MarkMotionClient::thrust_forward(double value, double timeout, WaitMode wm)
{
    MotionClient::thrust(Axis::TX, value, timeout, wm);
}

void MarkMotionClient::thrust_backward(double value, double timeout, WaitMode wm)
{
    MotionClient::thrust(Axis::TX, -value, timeout, wm);
}

void MarkMotionClient::thrust_right(double value, double timeout, WaitMode wm)
{
    MotionClient::thrust(Axis::MZ, value, timeout, wm);
}

void MarkMotionClient::thrust_left(double value, double timeout, WaitMode wm)
{
    MotionClient::thrust(Axis::MZ, -value, timeout, wm);
}

void MarkMotionClient::thrust_up(double value, double timeout, WaitMode wm)
{
    MotionClient::thrust(Axis::MY, value, timeout, wm);
}

void MarkMotionClient::thrust_down(double value, double timeout, WaitMode wm)
{
    MotionClient::thrust(Axis::MY, -value, timeout, wm);
}


void MarkMotionClient::fix_heading(double heading, double timeout, WaitMode wm)
{
    MotionClient::fix_heading(heading, timeout, wm);
}

void MarkMotionClient::turn_right(double bearing, double timeout, WaitMode wm)
{
    MotionClient::turn_right(bearing, timeout, wm);
}

void MarkMotionClient::turn_left(double bearing, double timeout, WaitMode wm)
{
    MotionClient::turn_left(bearing, timeout, wm);
}


void MarkMotionClient::fix_pitch(double value, double timeout, WaitMode wm)
{
    MotionClient::fix_pitch(value, timeout, wm);
}

void MarkMotionClient::turn_up(double value, double timeout, WaitMode wm)
{
    MotionClient::turn_up(value, timeout, wm);
}

void MarkMotionClient::turn_down(double value, double timeout, WaitMode wm)
{
    MotionClient::turn_down(value, timeout, wm);
}


void MarkMotionClient::fix_position(Point2d value, double timeout, WaitMode wm)
{
    MotionClient::fix_position(value, MoveMode::CRUISE, timeout, wm);
}

void MarkMotionClient::move_forward(double value, double timeout, WaitMode wm)
{
    MotionClient::move_forward(value, MoveMode::CRUISE, timeout, wm);
}

void MarkMotionClient::move_backward(double value, double timeout, WaitMode wm)
{
    MotionClient::move_backward(value, MoveMode::CRUISE, timeout, wm);
}

void MarkMotionClient::move_right(double value, double timeout, WaitMode wm)
{
    MotionClient::move_right(value, MoveMode::CRUISE, timeout, wm);
}

void MarkMotionClient::move_left(double value, double timeout, WaitMode wm)
{
    MotionClient::move_left(value, MoveMode::CRUISE, timeout, wm);
}


void MarkMotionClient::fix_velocity(double value, double timeout, WaitMode wm)
{
    MotionClient::fix_velocity(value, timeout, wm);
}

void MarkMotionClient::fix_vert(double value, SpeedyVertMode mode, double timeout, WaitMode wm)
{
    MotionClient::fix_vert(value, mode, timeout, wm);
}
