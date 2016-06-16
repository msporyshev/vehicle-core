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
    height_pub_       = comm.advertise<dvl::MsgHeight>();
    distance_pub_     = comm.advertise<dvl::MsgDistance>();
    velocity_pub_     = comm.advertise<dvl::MsgVelocity>();
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
    
    timer_data_publish_heigth_      = comm.create_timer(PERIOD_PUBLISH, &Dvl::publish_height, this);
    timer_data_publish_distance_    = comm.create_timer(PERIOD_PUBLISH, &Dvl::publish_distance, this);
    timer_data_publish_velocity_    = comm.create_timer(PERIOD_PUBLISH, &Dvl::publish_velocity, this);
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
        heigth_.heigth = earth_dist.range;
        heigth_.is_new = true;
        ROS_DEBUG_STREAM("Earth ref distance refreshed");
    } else {
        ROS_DEBUG_STREAM("Earth ref distance not refreshed");
    }
    
    if(dvl_trdi_.get_instrument_velocity(inst_vel, REF_TYPE_BOTTOMTRACK)) {
        velocity_.down     =  inst_vel.forward / 1000;
        velocity_.forward  = -inst_vel.rigthward / 1000;
        velocity_.right    =  inst_vel.downward / 1000;
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

    heigth_.heigth = test_data * 10;
    heigth_.is_new = true;

    distance_.backward  = test_data * 1; 
    distance_.forward   = test_data * 2; 
    distance_.leftward  = test_data * 3; 
    distance_.rightward = test_data * 4; 

    velocity_.down      = test_data * 0.1;
    velocity_.forward   = test_data * 0.2; 
    velocity_.right     = test_data * 0.3;
    velocity_.is_new = true;
    ROS_DEBUG_STREAM("Updated modelling data");
}

void Dvl::publish_height(const ros::TimerEvent& event)
{
    dvl::MsgHeight msg;
    if(heigth_.is_new) {
        msg.header.stamp = ros::Time::now();
        msg.height = heigth_.heigth;
        ROS_DEBUG_STREAM("Published " << ipc::classname(msg));
        height_pub_.publish(msg);
        heigth_.is_new = false;
    } else {
        ROS_DEBUG_STREAM("height data is not refreshed");
    }
}

void Dvl::publish_distance(const ros::TimerEvent& event)
{
    dvl::MsgDistance msg;
    
    if(distance_.is_new) {
        msg.header.stamp = ros::Time::now();
        msg.distance_backward  = distance_.backward;
        msg.distance_forward   = distance_.forward;
        msg.distance_leftward  = distance_.leftward;
        msg.distance_rightward = distance_.rightward;
        ROS_DEBUG_STREAM("Published " << ipc::classname(msg));
        distance_pub_.publish(msg);
        distance_.is_new = false;
    } else {
        ROS_DEBUG_STREAM("distance data is not refreshed");
    }

}

void Dvl::publish_velocity(const ros::TimerEvent& event)
{
    dvl::MsgVelocity msg;
    
    if(velocity_.is_new) {
        msg.header.stamp = ros::Time::now();
        msg.velocity_down    = velocity_.down;
        msg.velocity_forward = velocity_.forward;
        msg.velocity_right   = velocity_.right;
        ROS_DEBUG_STREAM("Published " << ipc::classname(msg));
        velocity_pub_.publish(msg);
        velocity_.is_new = false;
    } else {
        ROS_DEBUG_STREAM("velocity data is not refreshed");
    } 
}

///@}