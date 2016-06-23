/**
\file
\brief Драйвер доплера

В данном файле находятся реализации методов, объявленных в dvl.h

\ingroup dvl_node
*/

///@{

#include <iostream>
#include <fstream>

#include "dvl/dvl.h"

#define PERIOD_UPDATE       0.05
#define PERIOD_PUBLISH      0.10

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
    downward_distance_pub_ = comm.advertise<dvl::MsgDownwardDistance>();
    downward_velocity_pub_ = comm.advertise<dvl::MsgDownwardVelocity>();
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

    timer_pub_distance_    = comm.create_timer(PERIOD_PUBLISH, &Dvl::publish_distance, this);
    timer_pub_velocity_    = comm.create_timer(PERIOD_PUBLISH, &Dvl::publish_velocity, this);
}

void Dvl::print_header()
{

}

void Dvl::print_sensors_info()
{

}

void Dvl::data_update(const ros::TimerEvent& event)
{
    bool status = dvl_trdi_.dvl_process();
    if(status) {
        ROS_WARN_STREAM("Com data not received or data is incorrect");
    } else {
        ROS_DEBUG_STREAM("Com data received");
    }

    earth_reference_dist earth_dist;
    instrument_reference_vel inst_vel;

    if(dvl_trdi_.get_earth_distance(earth_dist, REF_TYPE_BOTTOMTRACK)) {
        distance_.downward  = earth_dist.range;
        distance_.is_new = true;
        ROS_DEBUG_STREAM("Earth ref distance refreshed");
    } else {
        ROS_DEBUG_STREAM("Earth ref distance not refreshed");
    }
    
    if(dvl_trdi_.get_instrument_velocity(inst_vel, REF_TYPE_BOTTOMTRACK)) {
        velocity_.downward  =  inst_vel.downward / 1000;
        velocity_.forward   = -inst_vel.forward / 1000;
        velocity_.rightward =  inst_vel.rightward / 1000;
        velocity_.is_new = true;
        ROS_DEBUG_STREAM("Earth ref distance refreshed");
    } else {
        ROS_DEBUG_STREAM("Instrument ref velocity not refreshed");
    }
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
    distance_.rightward = test_data * 2; 

    velocity_.downward  = test_data * 0.1;
    velocity_.forward   = test_data * 0.2; 
    velocity_.rightward = test_data * 0.3;
    velocity_.is_new = true;
    ROS_DEBUG_STREAM("Updated modelling data");
}

void Dvl::publish_distance(const ros::TimerEvent& event)
{
    dvl::MsgDownwardDistance msg;
    
    if(distance_.is_new) {
        msg.header.stamp = ros::Time::now();
        msg.downward = distance_.downward;
        ROS_DEBUG_STREAM("Published " << ipc::classname(msg));
        downward_distance_pub_.publish(msg);
        distance_.is_new = false;
    } else {
        ROS_DEBUG_STREAM("distance data is not refreshed");
    }

}

void Dvl::publish_velocity(const ros::TimerEvent& event)
{
    dvl::MsgPlaneVelocity msg_plane;
    dvl::MsgDownwardVelocity msg_down;
    
    if(velocity_.is_new) {        
        msg_plane.forward     = velocity_.forward;
        msg_plane.rightward   = velocity_.rightward;
        ROS_DEBUG_STREAM("Published " << ipc::classname(msg_plane));
        plane_velocity_pub_.publish(msg_plane);
        velocity_.is_new = false;

        msg_down.header.stamp = ros::Time::now();
        msg_down.downward = velocity_.downward;
        ROS_DEBUG_STREAM("Published " << ipc::classname(msg_down));
        downward_velocity_pub_.publish(msg_down);
        velocity_.is_new = false;

    } else {
        ROS_DEBUG_STREAM("velocity data is not refreshed");
    } 
}

///@}