#pragma once

#include <libauv/point/point.h>

struct NavigInfo
{
    double heading;
    double pitch;
    double roll;
    double velocity_forward;
    double velocity_depth;
    double velocity_north;
    double velocity_east;
    double heading_rate;
    double pitch_rate;
    double roll_rate;
    double depth;
    double height;
    libauv::Point2f position;
};