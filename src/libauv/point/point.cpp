#include <cmath>

#include "point/point.h"

#define REGISTER_POINT2(MsgPoint, ValueType) \
    MsgPoint MakePoint2(ValueType x, ValueType y) \
    { \
        MsgPoint pnt; \
        pnt.x = x; \
        pnt.y = y; \
        return pnt; \
    } \
    \
    double norm(const MsgPoint &p) \
    { \
        return std::sqrt((double)p.x*p.x + (double)p.y*p.y); \
    } \
    \
    MsgPoint operator+(const MsgPoint &p1, const MsgPoint &p2) \
    { \
        MsgPoint pnt; \
        pnt.x = p1.x + p2.x; \
        pnt.y = p1.y + p2.y; \
        return pnt; \
    } \
    \
    MsgPoint operator-(const MsgPoint &p1, const MsgPoint &p2) \
    { \
        MsgPoint pnt; \
        pnt.x = p1.x - p2.x; \
        pnt.y = p1.y - p2.y; \
        return pnt; \
    } \
    \
    MsgPoint operator*(const MsgPoint &p, ValueType k) \
    { \
        MsgPoint pnt; \
        pnt.x = p.x * k; \
        pnt.y = p.y * k; \
        return pnt; \
    } \
    \
    MsgPoint& operator+=(MsgPoint &p1, const MsgPoint &p2) \
    { \
        p1.x += p2.x; \
        p1.y += p2.y; \
        return p1; \
    } \
    \
    MsgPoint& operator-=(MsgPoint &p1, const MsgPoint &p2) \
    { \
        p1.x -= p2.x; \
        p1.y -= p2.y; \
        return p1; \
    } \
    \
    MsgPoint& operator*=(MsgPoint &p, ValueType k) \
    { \
        return p = p * k; \
    } \
    \
    bool operator==(const MsgPoint &p1, const MsgPoint &p2) \
    { \
        if (p1.x == p2.x && p1.y == p2.y) \
            return true; \
        return false; \
    } \
    \
    bool operator!=(const MsgPoint &p1, const MsgPoint &p2) \
    { \
        return !(p1 == p2); \
    } \

#define REGISTER_POINT3(MsgPoint, ValueType) \
    MsgPoint MakePoint3(ValueType x, ValueType y, ValueType z) \
    { \
        MsgPoint pnt; \
        pnt.x = x; \
        pnt.y = y; \
        pnt.z = z; \
        return pnt; \
    } \
    \
    double norm(const MsgPoint &p) \
    { \
        return std::sqrt((double)p.x*p.x + (double)p.y*p.y + (double)p.z*p.z); \
    } \
    \
    MsgPoint operator+(const MsgPoint &p1, const MsgPoint &p2) \
    { \
        MsgPoint pnt; \
        pnt.x = p1.x + p2.x; \
        pnt.y = p1.y + p2.y; \
        pnt.z = p1.z + p2.z; \
        return pnt; \
    } \
    \
    MsgPoint operator-(const MsgPoint &p1, const MsgPoint &p2) \
    { \
        MsgPoint pnt; \
        pnt.x = p1.x - p2.x; \
        pnt.y = p1.y - p2.y; \
        pnt.z = p1.z - p2.z; \
        return pnt; \
    } \
    \
    MsgPoint operator*(const MsgPoint &p, ValueType k) \
    { \
        MsgPoint pnt; \
        pnt.x = p.x * k; \
        pnt.y = p.y * k; \
        pnt.z = p.z * k; \
        return pnt; \
    } \
    \
    MsgPoint& operator+=(MsgPoint &p1, const MsgPoint &p2) \
    { \
        p1.x += p2.x; \
        p1.y += p2.y; \
        p1.z += p2.z; \
        return p1; \
    } \
    \
    MsgPoint& operator-=(MsgPoint &p1, const MsgPoint &p2) \
    { \
        p1.x -= p2.x; \
        p1.y -= p2.y; \
        p1.z -= p2.z; \
        return p1; \
    } \
    \
    MsgPoint& operator*=(MsgPoint &p, ValueType k) \
    { \
        return p = p * k; \
    } \
    \
    bool operator==(const MsgPoint &p1, const MsgPoint &p2) \
    { \
        if (p1.x == p2.x && p1.y == p2.y && p1.z == p2.z) \
            return true; \
        return false; \
    } \
    \
    bool operator!=(const MsgPoint &p1, const MsgPoint &p2) \
    { \
        return !(p1 == p2); \
    } \

REGISTER_POINT2(Point2i, int)
REGISTER_POINT2(Point2f, float)
REGISTER_POINT2(Point2d, double)

REGISTER_POINT3(Point3i, int)
REGISTER_POINT3(Point3f, float)
REGISTER_POINT3(Point3d, double)
