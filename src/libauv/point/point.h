#ifndef POINT_H
#define POINT_H

#include "libauv/Point2i.h"
#include "libauv/Point2f.h"
#include "libauv/Point2d.h"
#include "libauv/Point3i.h"
#include "libauv/Point3f.h"
#include "libauv/Point3d.h"

#include <ostream>

#define REGISTER_POINT2_HEADER(MsgPoint, ValueType) \
    MsgPoint MakePoint2(ValueType x, ValueType y); \
    double norm(const MsgPoint&); \
    MsgPoint operator+(const MsgPoint &p1, const MsgPoint &p2); \
    MsgPoint operator-(const MsgPoint &p1, const MsgPoint &p2); \
    MsgPoint operator*(const MsgPoint &p, ValueType k); \
    MsgPoint& operator+=(MsgPoint &p1, const MsgPoint &p2); \
    MsgPoint& operator-=(MsgPoint &p1, const MsgPoint &p2); \
    MsgPoint& operator*=(MsgPoint &p, ValueType k); \
    bool operator==(const MsgPoint &p1, const MsgPoint &p2); \
    bool operator!=(const MsgPoint &p1, const MsgPoint &p2); \

#define REGISTER_POINT3_HEADER(MsgPoint, ValueType) \
    MsgPoint MakePoint3(ValueType x, ValueType y, ValueType z); \
    double norm(const MsgPoint&); \
    MsgPoint operator+(const MsgPoint &p1, const MsgPoint &p2); \
    MsgPoint operator-(const MsgPoint &p1, const MsgPoint &p2); \
    MsgPoint operator*(const MsgPoint &p, ValueType k); \
    MsgPoint& operator+=(MsgPoint &p1, const MsgPoint &p2); \
    MsgPoint& operator-=(MsgPoint &p1, const MsgPoint &p2); \
    MsgPoint& operator*=(MsgPoint &p, ValueType k); \
    bool operator==(const MsgPoint &p1, const MsgPoint &p2); \
    bool operator!=(const MsgPoint &p1, const MsgPoint &p2); \

REGISTER_POINT2_HEADER(libauv::Point2i, int)
REGISTER_POINT2_HEADER(libauv::Point2f, float)
REGISTER_POINT2_HEADER(libauv::Point2d, double)

REGISTER_POINT3_HEADER(libauv::Point3i, int)
REGISTER_POINT3_HEADER(libauv::Point3f, float)
REGISTER_POINT3_HEADER(libauv::Point3d, double)

#endif