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

    navig::MsgLocalPosition bottom_target_pos(double real_size, double frame_size, Point2d frame_coord)
    {
        double k = real_size / frame_size;

        auto relative_pos = frame_coord * k;
        auto x = relative_pos.y; // TODO разобраться с системами координат
        auto y = relative_pos.x;

        auto local_pos = this->frame_pos();

        Point2d delta = Point2d((x * cos(utils::to_rad(frame_head())) - y * sin(utils::to_rad(frame_head()))),
            (x * sin(utils::to_rad(frame_head())) + y * cos(utils::to_rad(frame_head()))));

        local_pos.north += delta.x;
        local_pos.east += delta.y;
        return local_pos;
    }

    navig::MsgLocalPosition front_target_pos(double real_size, Point2d start, Point2d end, Point2d target)
    {
        double dist = front_camera_.calc_dist_to_object(real_size, start, end);

        double heading = frame_head();
        navig::MsgLocalPosition pos = frame_pos();
        pos.north += dist * cos(utils::to_rad(heading));
        pos.east += dist * sin(utils::to_rad(heading));

        return pos;
    }


private:
    ipc::Subscriber<navig::MsgOdometry> odometry_sub_;

    CameraModel front_camera_ = CameraModel::create_front_camera();
    CameraModel bottom_camera_ = CameraModel::create_bottom_camera();

    navig::MsgOdometry frame_odometry_ = navig::MsgOdometry();
};