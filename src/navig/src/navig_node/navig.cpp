#include "navig.h"

/**
\file
\brief Реализация навигационного модуля

В данном файле находятся реализации методов, объявленных в navig.h
Также здесь находится main

\defgroup navig_node Навигация
\brief Данный нод предназначен для реализации системы глобальной и локальной
навигации АНПА
*/

///@{
#include <navig/MsgNavigAccelerations.h>
#include <navig/MsgNavigDepth.h>
#include <navig/MsgNavigHeight.h>
#include <navig/MsgNavigPosition.h>
#include <navig/MsgNavigRates.h>

#include <map>
#include <string>

std::string Navig::get_name() const
{
    return NODE_NAME;
}

void Navig::init_ipc(ipc::Communicator& communicator)
{    
    /**
        Это регистрация всех исходящих сообщений навига
    */
    acc_pub_ = communicator.advertise<navig::MsgNavigAccelerations>();
    angles_pub_ = communicator.advertise<navig::MsgNavigAngles>();
    depth_pub_ = communicator.advertise<navig::MsgNavigDepth>();
    height_pub_ = communicator.advertise<navig::MsgNavigHeight>(); 
    position_pub_ = communicator.advertise<navig::MsgNavigPosition>(); 
    rates_pub_ = communicator.advertise<navig::MsgNavigRates>(); 
    velocity_pub_ = communicator.advertise<navig::MsgNavigVelocity>();

    /**
        Это подписка на все входящие сообщения навига
    */
    imu_angle_ = communicator.subscribe<compass::MsgCompassAngle>("compass");
    imu_acc_ = communicator.subscribe<compass::MsgCompassAcceleration>("compass");
    imu_rate_ = communicator.subscribe<compass::MsgCompassAngleRate>("compass");
    est_position_ = communicator.subscribe<navig::MsgEstimatedPosition>("local_position_estimator");
    dvl_dist_ = communicator.subscribe<dvl::MsgDvlDistance>("dvl");
    dvl_vel_ = communicator.subscribe<dvl::MsgDvlVelocity>("dvl");
    dvl_height_ = communicator.subscribe<dvl::MsgDvlHeight>("dvl");
    gps_coord_ = communicator.subscribe<gps::MsgGpsCoordinate>("gps");
    gps_sat_ = communicator.subscribe<gps::MsgGpsSatellites>("gps");
    gps_utc_ = communicator.subscribe<gps::MsgGpsUtc>("gps");
    supervisor_depth_ = communicator.subscribe<supervisor::MsgSupervisorDepth>("supervisor");
}

void Navig::run()
{
    ipc::EventLoop loop(get_period());
    while(loop.ok()) {
        if (imu_angle_.ready()) {
            handle_angles(*imu_angle_.msg());
        }

        if (imu_acc_.ready()) {
            handle_acceleration(*imu_acc_.msg());
        }

        if (imu_rate_.ready()) {
            handle_rate(*imu_rate_.msg());
        }

        if (supervisor_depth_.ready()) {
            handle_depth(*supervisor_depth_.msg());
        }

        if (est_position_.ready()) {
            handle_position(*est_position_.msg());
        }

        if (dvl_dist_.ready()) {
            NavigBase::handle_message(*dvl_dist_.msg());
        }

        if (dvl_vel_.ready()) {
            handle_velocity(*dvl_vel_.msg());
        }

        if (dvl_height_.ready()) {
            NavigBase::handle_message(*dvl_height_.msg());
        }

        if (gps_coord_.ready()) {
            NavigBase::handle_message(*gps_coord_.msg());
        }

        if (gps_sat_.ready()) {
            NavigBase::handle_message(*gps_sat_.msg());
        }

        if (gps_utc_.ready()) {
            NavigBase::handle_message(*gps_utc_.msg());
        }
    }
}

int Navig::get_period() const
{
    return delta_t_;
}

void Navig::handle_angles(const compass::MsgCompassAngle& msg)
{
    if (!check_time(msg)) {
        ROS_INFO_STREAM("Received too old message: " << ipc::classname(msg));
        return;
    }

    angles_data_.heading = msg.heading;
    angles_data_.roll = msg.roll;
    angles_data_.pitch = msg.pitch;

    ROS_INFO_STREAM("Published " << ipc::classname(angles_data_));
    angles_pub_.publish(angles_data_);
}

void Navig::handle_acceleration(const compass::MsgCompassAcceleration& msg)
{
    if (!check_time(msg)) {
        ROS_INFO_STREAM("Received too old message: " << ipc::classname(msg));
        return;
    }

    navig::MsgNavigAccelerations m;
    m.acc_x = msg.acc_x;
    m.acc_y = msg.acc_y;
    m.acc_z = msg.acc_z;

    ROS_INFO_STREAM("Published " << ipc::classname(m));
    acc_pub_.publish(m);
}

void Navig::handle_rate(const compass::MsgCompassAngleRate& msg)
{
    if (!check_time(msg)) {
        ROS_INFO_STREAM("Received too old message: " << ipc::classname(msg));
        return;
    }

    navig::MsgNavigRates m;
    m.rate_heading = msg.rate_head;
    m.rate_roll = msg.rate_roll;
    m.rate_pitch = msg.rate_pitch;
    
    ROS_INFO_STREAM("Published " << ipc::classname(m));
    rates_pub_.publish(m);   
}

void Navig::handle_position(const navig::MsgEstimatedPosition& msg)
{
    if (!check_time(msg)) {
        ROS_INFO_STREAM("Received too old message: " << ipc::classname(msg));
        return;
    }

    navig::MsgNavigPosition m;
    m.x = msg.x;
    m.y = msg.y;

    ROS_INFO_STREAM("Published " << ipc::classname(m));
    position_pub_.publish(m);
}

void Navig::handle_depth(const supervisor::MsgSupervisorDepth& msg) 
{
    if (!check_time(msg)) {
        ROS_INFO_STREAM("Received too old message: " << ipc::classname(msg));
        return;
    }

    navig::MsgNavigDepth m;
    m.depth = msg.depth;

    depth_pub_.publish(m);

    double new_depth_time = ipc::timestamp(msg);
    double time_diff(0.0);
    if (old_depth_time_ != 0) {
        time_diff = new_depth_time - old_depth_time_; 
    }
    old_depth_time_ = new_depth_time;
    velocity_data_.velocity_depth = calc_depth_velocity(m.depth, time_diff);
    send_velocity();
}

void Navig::handle_velocity(const dvl::MsgDvlVelocity& msg)
{
    if (!check_time(msg)) {
        ROS_INFO_STREAM("Received too old message: " << ipc::classname(msg));
        return;
    }

    velocity_data_.velocity_forward = msg.velocity_forward;
    velocity_data_.velocity_right = msg.velocity_right;
    velocity_data_.velocity_down = msg.velocity_down;
    send_velocity();
}

double Navig::calc_depth_velocity(double depth, double time_diff)
{
    if (time_diff == 0) {
        return velocity_data_.velocity_depth;
    }

    double new_depth = old_depth_ * 0.9 + depth * 0.1;
    velocity_data_.velocity_depth = (new_depth -  old_depth_) / time_diff;
    old_depth_ = new_depth;

    return velocity_data_.velocity_depth; 
}

void Navig::send_velocity()
{
    velocity_data_.velocity_north = velocity_data_.velocity_forward * cos(angles_data_.heading) 
        - velocity_data_.velocity_right * sin(angles_data_.heading);
    velocity_data_.velocity_east = velocity_data_.velocity_forward * sin(angles_data_.heading) 
        + velocity_data_.velocity_right * cos(angles_data_.heading);
    velocity_pub_.publish(velocity_data_);
}

///@}