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
#include <navig/MsgNavigAngles.h>
#include <navig/MsgNavigDepth.h>
#include <navig/MsgNavigHeight.h>
#include <navig/MsgNavigPosition.h>
#include <navig/MsgNavigRates.h>
#include <navig/MsgNavigVelocity.h>

using namespace std;

namespace navig
{

ros::Publisher acc_pub;
ros::Publisher angles_pub;
ros::Publisher depth_pub;
ros::Publisher height_pub;
ros::Publisher position_pub;
ros::Publisher rates_pub;
ros::Publisher velocity_pub;

void init_ipc(ipc::Communicator& communicator)
{    
    /**
        Это регистрация всех исходящих сообщений навига
    */
    acc_pub = communicator.advertise<navig::MsgNavigAccelerations>();
    angles_pub = communicator.advertise<navig::MsgNavigAngles>();
    depth_pub = communicator.advertise<navig::MsgNavigDepth>();
    height_pub = communicator.advertise<navig::MsgNavigHeight>(); 
    position_pub = communicator.advertise<navig::MsgNavigPosition>(); 
    rates_pub = communicator.advertise<navig::MsgNavigRates>(); 
    velocity_pub = communicator.advertise<navig::MsgNavigVelocity>();

    /**
        Это подписка на сообщения
    */
    communicator.subscribe("compass", handle_angles);
    communicator.subscribe("compass", handle_acceleration);
    communicator.subscribe("compass", handle_rate);
    communicator.subscribe("Local_position_estimator", handle_position);
    communicator.subscribe("dvl", handle_message<dvl::MsgDvlDistance>);
    communicator.subscribe("dvl", handle_message<dvl::MsgDvlVelocity>);
    communicator.subscribe("dvl", handle_message<dvl::MsgDvlHeight>);
    communicator.subscribe("gps", handle_message<gps::MsgGpsCoordinate>);
    communicator.subscribe("gps", handle_message<gps::MsgGpsSatellites>);
    communicator.subscribe("gps", handle_message<gps::MsgGpsUtc>);
    communicator.subscribe("supervisor", handle_message<supervisor::MsgSupervisorDepth>);
}

void handle_angles(const compass::MsgCompassAngle& msg)
{
    navig::MsgNavigAngles m;
    m.heading = msg.heading;
    m.roll = msg.roll;
    m.pitch = msg.pitch;

    ROS_INFO_STREAM("Published " << ipc::classname(m));
    angles_pub.publish(m);
}

void handle_acceleration(const compass::MsgCompassAcceleration& msg)
{
    navig::MsgNavigAccelerations m;
    m.acc_x = msg.acc_x;
    m.acc_y = msg.acc_y;
    m.acc_z = msg.acc_z;

    ROS_INFO_STREAM("Published " << ipc::classname(m));
    acc_pub.publish(m);
}

void handle_rate(const compass::MsgCompassAngleRate& msg)
{
    navig::MsgNavigRates m;
    m.rate_heading = msg.rate_head;
    m.rate_roll = msg.rate_roll;
    m.rate_pitch = msg.rate_pitch;
    
    ROS_INFO_STREAM("Published " << ipc::classname(m));
    rates_pub.publish(m);   
}

void handle_position(const navig::MsgEstimatedPosition& msg)
{
    navig::MsgNavigPosition m;
    m.x = msg.x;
    m.y = msg.y;

    ROS_INFO_STREAM("Published " << ipc::classname(m));
    position_pub.publish(m);
}

// void handle_distance_forward(const dvl::MsgDvlDistanceForward& msg) {}

// void handle_distance_leftward(const dvl::MsgDvlDistanceLeftward& msg) {}

// void handle_distance_rightward(const dvl::MsgDvlDistanceRightward& msg) {}

// void handle_velocity_down(const dvl::MsgDvlVelocityDown& msg) {}

// void handle_velocity_forward(const dvl::MsgDvlVelocityForward& msg) {}

// void handle_velocity_right(const dvl::MsgDvlVelocityRight& msg) {}

// void handle_height(const dvl::MsgDvlHeight& msg) {}

// void handle_coordinate(const gps::MsgGpsCoordinate& msg) {}

// void handle_satellites(const gps::MsgGpsSatellites& msg) {}

// void handle_utc(const gps::MsgGpsUtc& msg) {}

void handle_depth(const supervisor::MsgSupervisorDepth& msg) 
{
    navig::MsgNavigDepth m;
    m.depth = msg.depth;

    ROS_INFO_STREAM("Published " << ipc::classname(m));
    depth_pub.publish(m);
}

void create_and_publish_acc()
{
    navig::MsgNavigAccelerations msg;
    msg.acc_x = 10;
    msg.acc_y = 20;
    msg.acc_z = 3;
    acc_pub.publish(msg);   
}

void create_and_publish_angles()
{
    navig::MsgNavigAngles msg;
    msg.heading = 0;
    msg.roll = 45;
    msg.pitch = 60;
    angles_pub.publish(msg);
}

void create_and_publish_rates()
{
    navig::MsgNavigRates msg;
    msg.rate_heading = 3;
    msg.rate_roll = 4;
    msg.rate_pitch = 5;
    rates_pub.publish(msg);   
}

void create_and_publish_depth()
{
    navig::MsgNavigDepth msg;
    msg.depth = 1.0;
    ROS_INFO_STREAM("Published " << ipc::classname(msg));
    depth_pub.publish(msg);
}

void create_and_publish_height()
{
    navig::MsgNavigHeight msg;
    msg.height = 1.0;
    ROS_INFO_STREAM("Published " << ipc::classname(msg));
    height_pub.publish(msg);
}

void create_and_publish_position()
{
    navig::MsgNavigPosition msg;
    msg.lon = 131;
    msg.lat = 43;
    msg.x = 1;
    msg.y = -1;
    ROS_INFO_STREAM("Published " << ipc::classname(msg));
    position_pub.publish(msg);
}

void create_and_publish_velocity()
{
    navig::MsgNavigVelocity msg;
    msg.velocity_forward = 1.0;
    msg.velocity_right = 0.5;
    msg.velocity_down = 0.1;
    ROS_INFO_STREAM("Published " << ipc::classname(msg));
    velocity_pub.publish(msg);
}

}

///@}