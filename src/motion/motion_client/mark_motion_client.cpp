#include "mark_motion_client.h"
#include "move_mode.h"

using namespace std;

void MarkMotionClient::thrust_forward(float value, float timeout, WaitMode wm)
{
    MotionClient::thrust(Axis::TX, value, timeout, wm);
}

void MarkMotionClient::thrust_backward(float value, float timeout, WaitMode wm)
{
    MotionClient::thrust(Axis::TX, -value, timeout, wm);
}

void MarkMotionClient::thrust_right(float value, float timeout, WaitMode wm)
{
    MotionClient::thrust(Axis::MZ, value, timeout, wm);
}

void MarkMotionClient::thrust_left(float value, float timeout, WaitMode wm)
{
    MotionClient::thrust(Axis::MZ, -value, timeout, wm);
}

void MarkMotionClient::thrust_up(float value, float timeout, WaitMode wm)
{
    MotionClient::thrust(Axis::MY, value, timeout, wm);
}

void MarkMotionClient::thrust_down(float value, float timeout, WaitMode wm)
{
    MotionClient::thrust(Axis::MY, -value, timeout, wm);
}


void MarkMotionClient::fix_heading(float heading, float timeout, WaitMode wm)
{
    MotionClient::fix_heading(heading, timeout, wm);
}

void MarkMotionClient::turn_right(float bearing, float timeout, WaitMode wm)
{
    MotionClient::turn_right(bearing, timeout, wm);
}

void MarkMotionClient::turn_left(float bearing, float timeout, WaitMode wm)
{
    MotionClient::turn_left(bearing, timeout, wm);
}


void MarkMotionClient::fix_pitch(float value, float timeout, WaitMode wm)
{
    MotionClient::fix_pitch(value, timeout, wm);
}

void MarkMotionClient::turn_up(float value, float timeout, WaitMode wm)
{
    MotionClient::turn_up(value, timeout, wm);
}

void MarkMotionClient::turn_down(float value, float timeout, WaitMode wm)
{
    MotionClient::turn_down(value, timeout, wm);
}


void MarkMotionClient::fix_position(libauv::Point2d value, float timeout, WaitMode wm)
{
    MotionClient::fix_position(value, MoveMode::CRUISE, timeout, wm);
}

void MarkMotionClient::move_forward(float value, float timeout, WaitMode wm)
{
    MotionClient::move_forward(value, MoveMode::CRUISE, timeout, wm);
}

void MarkMotionClient::move_backward(float value, float timeout, WaitMode wm)
{
    MotionClient::move_backward(value, MoveMode::CRUISE, timeout, wm);
}

void MarkMotionClient::move_right(float value, float timeout, WaitMode wm)
{
    MotionClient::move_right(value, MoveMode::CRUISE, timeout, wm);
}

void MarkMotionClient::move_left(float value, float timeout, WaitMode wm)
{
    MotionClient::move_left(value, MoveMode::CRUISE, timeout, wm);
}


void MarkMotionClient::fix_velocity(float value, float timeout, WaitMode wm)
{
    MotionClient::fix_velocity(value, timeout, wm);
}

void MarkMotionClient::fix_vert(float value, SpeedyVertMode mode, float timeout, WaitMode wm)
{
    MotionClient::fix_vert(value, mode, timeout, wm);
}
