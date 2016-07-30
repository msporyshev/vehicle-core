/**
\file
\brief Драйвер доплера

В данном файле находятся реализации методов, объявленных в dvl.h

\ingroup dvl_node
*/

///@{

#include <iostream>
#include <fstream>
#include <sstream>

#include "dvl/dvl.h"

#define PERIOD_UPDATE       0.01
#define PERIOD_PUBLISH      0.05

using namespace std;

const string Dvl::NODE_NAME = "dvl";

Dvl::Dvl(dvlConfig config): config_(config), new_data_avalible_(false)
{
    print_header();
}

void Dvl::init_connection(ipc::Communicator& comm)
{
    /**
        Регистрация всех исходящих сообщений доплера
    */
    down_pub_              = comm.advertise<dvl::MsgDown>();
    plane_velocity_pub_    = comm.advertise<dvl::MsgPlaneVelocity>();
}

void Dvl::init_dvl()
{
    if(config_.start_now) {
        ROS_INFO_STREAM("Dopler started");
        dvl_trdi_.dvl_start(config_.port.c_str(), config_.baudrate);
    }
}

void Dvl::deinit_dvl()
{
    ROS_INFO_STREAM("Dopler stoped");
    dvl_trdi_.dvl_stop();
}

void Dvl::start_timers(ipc::Communicator& comm)
{
    if(!config_.modelling) {
        ROS_INFO_STREAM("Activated work mode");
        timer_data_update_  = comm.create_timer(PERIOD_UPDATE, &Dvl::data_update, this);
    } else {
        ROS_INFO_STREAM("Activated modelling mode");
        timer_data_update_  = comm.create_timer(PERIOD_UPDATE, &Dvl::data_update_modelling, this);
    }

    timer_pub_down_             = comm.create_timer(PERIOD_PUBLISH, &Dvl::publish_down_data, this);
    timer_pub_plane_velocity_   = comm.create_timer(PERIOD_PUBLISH, &Dvl::publish_plane_velocity, this);
}

void Dvl::print_header()
{
    ROS_DEBUG_STREAM("Time[sec]" << "\t" << "Down distance[m]" << "\t"  << "Forward velocity[m/sec]" <<
        "Right velocity[m/sec]" << "\t" <<"Down velocity[m/sec]" << "\n");
}

void Dvl::print_data()
{
    std::stringstream ss;

    if(distance_.is_new) {
        ss << distance_.down;
    }
    ss << "\t";
    if(velocity_.is_new) {
        ss << velocity_.forward << "\t" << velocity_.right << "\t" 
                        << velocity_.down;
    }
    ROS_DEBUG_STREAM(ros::Time::now() << ss.str());
}

void Dvl::print_sensors_info()
{

}

void Dvl::data_update(const ros::TimerEvent& event)
{
    bool status = dvl_trdi_.dvl_process();
    if(status) {
        ROS_DEBUG_STREAM("Com data not received or data is incorrect");
    } else {
        ROS_DEBUG_STREAM("Com data received");
    }

    earth_reference_dist earth_dist;
    instrument_reference_vel inst_vel;

    if(!dvl_trdi_.get_earth_distance(earth_dist, REF_TYPE_BOTTOMTRACK)) {
        distance_.down  = earth_dist.range;
        down_.distance  = earth_dist.range;
        distance_.is_new = true;
        down_.is_distance_new = true;
        ROS_DEBUG_STREAM("Earth ref distance refreshed");
    } else {
        ROS_DEBUG_STREAM("Earth ref distance not refreshed");
    }

    if(!dvl_trdi_.get_instrument_velocity(inst_vel, REF_TYPE_BOTTOMTRACK)) {
        velocity_.down      =  inst_vel.down / 1000.0;
        velocity_.forward   =  inst_vel.forward / 1000.0;
        velocity_.right     = -inst_vel.right / 1000.0;
        down_.velocity      =  inst_vel.down / 1000.0;
        velocity_.is_new = true;
        down_.is_velocity_new = true;
        ROS_DEBUG_STREAM("Instrument ref velocity refreshed");
    } else {
        ROS_DEBUG_STREAM("Instrument ref velocity not refreshed");
    }
    print_data();
}

void Dvl::data_update_modelling(const ros::TimerEvent& event)
{
    static float test_data = 0;

    if (test_data > 10) {
        test_data = 0;
    } else {
        test_data++;
    }

    distance_.forward   = test_data * 1;
    distance_.right     = test_data * 2;
    distance_.is_new = true;

    down_.distance      = test_data * 1;
    down_.velocity      = test_data * 0.1;
    down_.is_velocity_new = true;
    down_.is_distance_new = true;

    velocity_.down      = test_data * 0.1;
    velocity_.forward   = test_data * 0.2;
    velocity_.right     = test_data * 0.3;
    velocity_.is_new = true;
    ROS_DEBUG_STREAM("Updated modelling data");
    print_data();
}

void Dvl::publish_down_data(const ros::TimerEvent& event)
{
    dvl::MsgDown msg;
    
    if(down_.is_distance_new && down_.is_velocity_new) {
        msg.header.stamp = ros::Time::now();
        msg.distance = down_.distance;
        msg.velocity = down_.velocity;
        ROS_DEBUG_STREAM("Published " << ipc::classname(msg));
        down_pub_.publish(msg);
        down_.is_distance_new = false;
        down_.is_velocity_new = false;
    } else {
        ROS_DEBUG_STREAM("distance data is not refreshed");
    }
}

void Dvl::publish_plane_velocity(const ros::TimerEvent& event)
{
    dvl::MsgPlaneVelocity msg_plane;

    if(velocity_.is_new) {
        msg_plane.header.stamp = ros::Time::now();
        msg_plane.forward   = velocity_.forward;
        msg_plane.right     = velocity_.right;
        ROS_DEBUG_STREAM("Published " << ipc::classname(msg_plane));
        plane_velocity_pub_.publish(msg_plane);
        velocity_.is_new = false;
    } else {
        ROS_DEBUG_STREAM("velocity data is not refreshed");
    }
}

///@}