#pragma once

#include <point/point.h>

struct NavigInfo
{
    double heading;
    double pitch;
    double roll;

    double velocity_forward;
    double velocity_right;

    double heading_rate;
    double pitch_rate;
    double roll_rate;

    double depth;
    double velocity_depth;
    double height;
    double velocity_height;
    Point2d position;
};