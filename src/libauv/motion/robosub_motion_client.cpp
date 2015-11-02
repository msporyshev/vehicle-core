#include "robosub_motion_client.h"
#include "ipc_lib.h"

using namespace std;

RobosubMotionClient::RobosubMotionClient()
{

}

RobosubMotionClient::~RobosubMotionClient()
{

}

void RobosubMotionClient::thrust_forward(double value, double timeout, WaitMode wm)
{
    thrust(Axis::TX, value, timeout, wm);
}

void RobosubMotionClient::thrust_backward(double value, double timeout, WaitMode wm)
{
    thrust(Axis::TX, -value, timeout, wm);
}

void RobosubMotionClient::thrust_right(double value, double timeout, WaitMode wm)
{
    thrust(Axis::TY, value, timeout, wm);
}

void RobosubMotionClient::thrust_left(double value, double timeout, WaitMode wm)
{
    thrust(Axis::TY, -value, timeout, wm);
}

void RobosubMotionClient::thrust_down(double value, double timeout, WaitMode wm)
{
    thrust(Axis::TZ, value, timeout, wm);
}

void RobosubMotionClient::thrust_up(double value, double timeout, WaitMode wm)
{
    thrust(Axis::TZ, -value, timeout, wm);
}


void RobosubMotionClient::fix_pitch()
{
    MotionClient::fix_pitch(0.0, FIX_TIMEOUT, WaitMode::DONT_WAIT);
}

void RobosubMotionClient::fix_heading(double heading, WaitMode wm)
{
    MotionClient::fix_heading(heading, FIX_TIMEOUT, wm);
}

void RobosubMotionClient::fix_heading(double heading, double timeout, WaitMode wm)
{
    MotionClient::fix_heading(heading, timeout, wm);
}

void RobosubMotionClient::turn_right(double bearing, WaitMode wm)
{
    MotionClient::turn_right(bearing, FIX_TIMEOUT, wm);
}

void RobosubMotionClient::turn_right(double bearing, double timeout, WaitMode wm)
{
    MotionClient::turn_right(bearing, timeout, wm);
}

void RobosubMotionClient::turn_left(double bearing, WaitMode wm)
{
    MotionClient::turn_left(bearing, FIX_TIMEOUT, wm);
}

void RobosubMotionClient::turn_left(double bearing, double timeout, WaitMode wm)
{
    MotionClient::turn_left(bearing, timeout, wm);
}

void RobosubMotionClient::fix_depth(double depth, WaitMode wm)
{
    MotionClient::fix_depth(depth, FIX_TIMEOUT, wm);
}

void RobosubMotionClient::fix_depth(double depth, double timeout, WaitMode wm)
{
    MotionClient::fix_depth(depth, timeout, wm);
}

void RobosubMotionClient::move_down(double depth, WaitMode wm)
{
    MotionClient::move_down(depth, FIX_TIMEOUT, wm);
}

void RobosubMotionClient::move_down(double depth, double timeout, WaitMode wm)
{
    MotionClient::move_down(depth, timeout, wm);
}

void RobosubMotionClient::move_up(double depth, WaitMode wm)
{
    MotionClient::move_up(depth, FIX_TIMEOUT, wm);
}

void RobosubMotionClient::move_up(double depth, double timeout, WaitMode wm)
{
    MotionClient::move_up(depth, timeout, wm);
}


void RobosubMotionClient::adjust(double thrust_forward, double thrust_right, double delta_heading, double delta_depth)
{
    MotionClient::thrust(Axis::TX, thrust_forward, ADJUST_TIMEOUT, WaitMode::DONT_WAIT);
    MotionClient::thrust(Axis::TY, thrust_right, ADJUST_TIMEOUT, WaitMode::DONT_WAIT);
    MotionClient::turn_right(delta_heading, ADJUST_TIMEOUT, WaitMode::DONT_WAIT);
    MotionClient::move_down(delta_depth, ADJUST_TIMEOUT, WaitMode::DONT_WAIT);
}


void RobosubMotionClient::unfix_all()
{
    MotionClient::unfix_all();
}

void RobosubMotionClient::fix_position(Point2d value, MoveMode move_mode, double timeout, WaitMode wm)
{
    MotionClient::fix_position(value, move_mode, timeout, wm);
}

void RobosubMotionClient::unseat(Point2d value, MoveMode move_mode, double timeout, WaitMode wm)
{
    MotionClient::unseat(value, move_mode, timeout, wm);
}

void RobosubMotionClient::move_forward(double value, double timeout, WaitMode wm)
{
    MotionClient::move_forward(value, MoveMode::HOVER, timeout, wm);
}

void RobosubMotionClient::move_backward(double value, double timeout, WaitMode wm)
{
    MotionClient::move_backward(value, MoveMode::HOVER, timeout, wm);
}

void RobosubMotionClient::move_right(double value, double timeout, WaitMode wm)
{
    MotionClient::move_right(value, MoveMode::HOVER, timeout, wm);
}

void RobosubMotionClient::move_left(double value, double timeout, WaitMode wm)
{
    MotionClient::move_left(value, MoveMode::HOVER, timeout, wm);
}
