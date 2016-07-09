#pragma once

#include "navig/MsgPoint2d.h"
#include "navig/MsgPoint2f.h"
#include "navig/MsgPlaneVelocity.h"
#include "navig/MsgLocalPosition.h"

#include <cmath>
#include <ostream>

class Point2d
{
public:
    Point2d(): x(0), y(0) {}
    Point2d(double x, double y): x(x), y(y) {}

    Point2d(const navig::MsgPlaneVelocity& msg): x(msg.forward), y(msg.right) {}
    Point2d(const navig::MsgLocalPosition& msg): x(msg.north), y(msg.east) {}
    Point2d(const navig::MsgPoint2d& msg): x(msg.x), y(msg.y) {}

    double x;
    double y;

    operator navig::MsgPlaneVelocity() const
    {
        navig::MsgPlaneVelocity msg;
        msg.forward = x;
        msg.right = y;
        return msg;
    }

    operator navig::MsgLocalPosition() const
    {
        navig::MsgLocalPosition msg;
        msg.north = x;
        msg.east = y;
        return msg;
    }

    operator navig::MsgPoint2d() const {
        navig::MsgPoint2d p;
        p.x = x;
        p.y = y;
        return p;
    }
};


inline double norm(const Point2d& p)
{
    return std::sqrt(p.x*p.x + p.y*p.y);
}

inline Point2d operator+(const Point2d& p1, const Point2d& p2) {
    Point2d res;
    res.x = p1.x + p2.x;
    res.y = p1.y + p2.y;
    return res;
}

inline Point2d operator-(const Point2d& p1, const Point2d& p2) {
    Point2d res;
    res.x = p1.x - p2.x;
    res.y = p1.y - p2.y;
    return res;
}

inline Point2d operator*(const Point2d& p1, double k) {
    Point2d res;
    res.x = p1.x * k;
    res.y = p1.y * k;
    return res;
}

inline Point2d& operator+=(Point2d& p1, const Point2d& p2)
{
    p1.x += p2.x;
    p1.y += p2.y;
    return p1;
}

inline Point2d& operator-=(Point2d& p1, const Point2d& p2)
{
    p1.x -= p2.x;
    p1.y -= p2.y;
    return p1;
}

inline Point2d& operator*=(Point2d& p1, double k)
{
    p1.x *= k;
    p1.y *= k;
    return p1;
}

inline bool operator==(const Point2d& p1, const Point2d& p2)
{
    if (p1.x == p2.x && p1.y == p2.y)
        return true;
    return false;
}

inline bool operator!=(const Point2d& p1, const Point2d& p2)
{
    return !(p1 == p2);
}
