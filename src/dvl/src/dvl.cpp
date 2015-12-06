#include <iostream>
#include <fstream>

#include "dvl/dvl.h"

using namespace std;

const string Dvl::NODE_NAME = "dvl";

Dvl::Dvl()
{
    print_header();
}

void Dvl::init_connection(ipc::Communicator& comm)
{
    /**
        Регистрация всех исходящих сообщений навига
    */
    height_pub_       = comm.advertise<dvl::MsgDvlHeight>();
    distance_pub_     = comm.advertise<dvl::MsgDvlDistance>();
    velocity_pub_     = comm.advertise<dvl::MsgDvlVelocity>();
}

void Dvl::print_header()
{
}

void Dvl::print_sensors_info()
{

}

void Dvl::publish_height(const ros::TimerEvent& event)
{
    dvl::MsgDvlHeight msg;
    msg.height = 10.1;
    cout << "send MsgDvlHeight data" << endl;
    height_pub_.publish(msg);
}


void Dvl::publish_distance(const ros::TimerEvent& event)
{
    dvl::MsgDvlDistance msg;
    
    msg.distance_backward  = 10.2;
    msg.distance_forward   = 10.3;
    msg.distance_leftward  = 10.4;
    msg.distance_rightward = 10.5;
    
    cout << "send MsgDvlDistance data" << endl;
    distance_pub_.publish(msg);
}

void Dvl::publish_velocity(const ros::TimerEvent& event)
{
    dvl::MsgDvlVelocity msg;
    
    msg.velocity_down    = 3.1;
    msg.velocity_forward = 3.2;
    msg.velocity_right   = 3.3;
    
    cout << "send MsgDvlVelocity data" << endl;
    velocity_pub_.publish(msg);
}