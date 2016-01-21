#include "navig_simulating.h"

#include <navig/MsgNavigAccelerations.h>
#include <navig/MsgNavigAngles.h>
#include <navig/MsgNavigDepth.h>
#include <navig/MsgNavigHeight.h>
#include <navig/MsgNavigPosition.h>
#include <navig/MsgNavigRates.h>
#include <navig/MsgNavigVelocity.h>

std::string NavigSimulating::get_name() const
{
    return NODE_NAME;
} 

void NavigSimulating::init_ipc(ipc::Communicator& communicator)
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
}

void NavigSimulating::run()
{
    ipc::EventLoop loop(delta_t_);
    while (loop.ok()) {
        create_and_publish_acc();
        create_and_publish_angles();
        create_and_publish_rates();
        create_and_publish_depth();
        create_and_publish_height();
        create_and_publish_position();
        create_and_publish_velocity();
    }
}

void NavigSimulating::create_and_publish_acc()
{
    navig::MsgNavigAccelerations msg;
    msg.acc_x = 10;
    msg.acc_y = 20;
    msg.acc_z = 3;
    acc_pub_.publish(msg);   
}

void NavigSimulating::create_and_publish_angles()
{
    navig::MsgNavigAngles msg;
    msg.heading = 0;
    msg.roll = 45;
    msg.pitch = 60;
    angles_pub_.publish(msg);
}

void NavigSimulating::create_and_publish_rates()
{
    navig::MsgNavigRates msg;
    msg.rate_heading = 3;
    msg.rate_roll = 4;
    msg.rate_pitch = 5;
    rates_pub_.publish(msg);   
}

void NavigSimulating::create_and_publish_depth()
{
    navig::MsgNavigDepth msg;
    msg.depth = 1.0;
    ROS_INFO_STREAM("Published " << ipc::classname(msg));
    depth_pub_.publish(msg);
}

void NavigSimulating::create_and_publish_height()
{
    navig::MsgNavigHeight msg;
    msg.height = 1.0;
    ROS_INFO_STREAM("Published " << ipc::classname(msg));
    height_pub_.publish(msg);
}

void NavigSimulating::create_and_publish_position()
{
    navig::MsgNavigPosition msg;
    msg.lon = 131;
    msg.lat = 43;
    msg.x = 1;
    msg.y = -1;
    ROS_INFO_STREAM("Published " << ipc::classname(msg));
    position_pub_.publish(msg);
}

void NavigSimulating::create_and_publish_velocity()
{
    navig::MsgNavigVelocity msg;
    msg.velocity_forward = 1.0;
    msg.velocity_right = 0.5;
    msg.velocity_down = 0.1;
    ROS_INFO_STREAM("Published " << ipc::classname(msg));
    velocity_pub_.publish(msg);
}
