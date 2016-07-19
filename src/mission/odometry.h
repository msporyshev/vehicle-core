#pragma once

#include <libipc/ipc.h>
#include <navig/MsgDepth.h>
#include <navig/MsgAngle.h>
#include <navig/MsgPlaneVelocity.h>
#include <navig/MsgOdometry.h>

#include <utils/utils.h>
#include <point/point.h>

class Odometry
{
public:
    Odometry(ipc::Communicator& comm)
            : odometry_sub_(comm.subscribe<navig::MsgOdometry>("navig"))
    {}

    void add_frame_odometry(const navig::MsgOdometry odometry) {
        frame_odometry_ = odometry;
        //TODO: возможно потом появится логика с хранением навигации для нескольких кадров
    }

    navig::MsgDepth frame_depth() {
        if (frame_odometry_.has_depth) {
            return frame_odometry_.depth;
        } else {
            return depth();
        }
    }

    navig::MsgHeight frame_height() {
        if (frame_odometry_.has_height) {
            return frame_odometry_.height;
        } else {
            return height();
        }
    }

    double frame_head() {
        if (frame_odometry_.has_angle) {
            return frame_odometry_.angle.heading;
        } else {
            return head();
        }
    }

    navig::MsgPlaneVelocity frame_velocity() {
        if (frame_odometry_.has_velocity) {
            return frame_odometry_.velocity;
        } else {
            return velocity();
        }
    }

    navig::MsgLocalPosition frame_pos() {
        if (frame_odometry_.has_pos) {
            return frame_odometry_.pos;
        } else {
            return pos();
        }
    }

    navig::MsgDepth depth()
    {
        return odometry_sub_.msg_wait().depth;
    }

    navig::MsgHeight height() {
        return odometry_sub_.msg_wait().height;
    }

    double head()
    {
        return odometry_sub_.msg_wait().angle.heading;
    }

    navig::MsgPlaneVelocity velocity()
    {
        return odometry_sub_.msg_wait().velocity;
    }

    navig::MsgLocalPosition pos()
    {
        return odometry_sub_.msg_wait().pos;
    }

private:
    ipc::Subscriber<navig::MsgOdometry> odometry_sub_;

    navig::MsgOdometry frame_odometry_ = navig::MsgOdometry();
};