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

#include <map>
#include <string>

using namespace std;

namespace navig
{

map<string, double> old_time = {
    { ipc::classname(compass::MsgCompassAngle()), 0.0 },
    { ipc::classname(compass::MsgCompassAcceleration()), 0.0 },
    { ipc::classname(compass::MsgCompassAngleRate()), 0.0 },
    { ipc::classname(navig::MsgEstimatedPosition()), 0.0 },
    { ipc::classname(dvl::MsgDvlDistance()), 0.0 },
    { ipc::classname(dvl::MsgDvlVelocity()), 0.0 },
    { ipc::classname(dvl::MsgDvlHeight()), 0.0 },
    { ipc::classname(gps::MsgGpsCoordinate()), 0.0 },
    { ipc::classname(gps::MsgGpsSatellites()), 0.0 },
    { ipc::classname(gps::MsgGpsUtc()), 0.0 },
    { ipc::classname(supervisor::MsgSupervisorDepth()), 0.0 }
};

ros::Publisher acc_pub;
ros::Publisher angles_pub;
ros::Publisher depth_pub;
ros::Publisher height_pub;
ros::Publisher position_pub;
ros::Publisher rates_pub;
ros::Publisher velocity_pub;

double old_depth_time = 0.0;
double old_depth = 0;

MsgNavigVelocity velocity_data;
MsgNavigAngles angles_data;

double timeout_old_data = 0.0;
int delta_t = 0;

double calc_depth_velocity(double depth, double time_diff);
void send_velocity();

template<typename MsgType>
bool check_time(const MsgType& msg)
{
    if (old_time[ipc::classname(msg)] == 0.0) {
        return true;
    }

    auto new_time = ros::message_traits::timeStamp(msg)->toSec(); 
    bool result = new_time - old_time[ipc::classname(msg)] <= timeout_old_data;
    old_time[ipc::classname(msg)] = new_time;

    return result;
}

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
}

void read_config(navig::NavigConfig& config, unsigned int level)
{
    timeout_old_data = config.timeout_old_data;
    delta_t = config.delta_t;
}

int get_period()
{
    return delta_t;
}

void handle_angles(const compass::MsgCompassAngle& msg)
{
    if (!check_time(msg)) {
        ROS_INFO_STREAM("Received too old message: " << ipc::classname(msg));
        return;
    }

    angles_data.heading = msg.heading;
    angles_data.roll = msg.roll;
    angles_data.pitch = msg.pitch;

    ROS_INFO_STREAM("Published " << ipc::classname(angles_data));
    angles_pub.publish(angles_data);
}

void handle_acceleration(const compass::MsgCompassAcceleration& msg)
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
    acc_pub.publish(m);
}

void handle_rate(const compass::MsgCompassAngleRate& msg)
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
    rates_pub.publish(m);   
}

void handle_position(const navig::MsgEstimatedPosition& msg)
{
    if (!check_time(msg)) {
        ROS_INFO_STREAM("Received too old message: " << ipc::classname(msg));
        return;
    }

    navig::MsgNavigPosition m;
    m.x = msg.x;
    m.y = msg.y;

    ROS_INFO_STREAM("Published " << ipc::classname(m));
    position_pub.publish(m);
}

void handle_depth(const supervisor::MsgSupervisorDepth& msg) 
{
    if (!check_time(msg)) {
        ROS_INFO_STREAM("Received too old message: " << ipc::classname(msg));
        return;
    }

    navig::MsgNavigDepth m;
    m.depth = msg.depth;

    depth_pub.publish(m);

    double new_depth_time = ros::message_traits::timeStamp(msg)->toSec();
    double time_diff(0.0);
    if (old_depth_time != 0) {
        time_diff = new_depth_time - old_depth_time; 
    }
    old_depth_time = new_depth_time;
    velocity_data.velocity_depth = calc_depth_velocity(m.depth, time_diff);
    send_velocity();
}

void handle_velocity(const dvl::MsgDvlVelocity& msg)
{
    if (!check_time(msg)) {
        ROS_INFO_STREAM("Received too old message: " << ipc::classname(msg));
        return;
    }

    velocity_data.velocity_forward = msg.velocity_forward;
    velocity_data.velocity_right = msg.velocity_right;
    velocity_data.velocity_down = msg.velocity_down;
    send_velocity();
}

double calc_depth_velocity(double depth, double time_diff)
{
    if (time_diff == 0) {
        return velocity_data.velocity_depth;
    }

    double new_depth = old_depth * 0.9 + depth * 0.1;
    velocity_data.velocity_depth = (new_depth -  old_depth) / time_diff;
    old_depth = new_depth;

    return velocity_data.velocity_depth; 
}

void send_velocity()
{
    velocity_data.velocity_north = velocity_data.velocity_forward * cos(angles_data.heading) 
        - velocity_data.velocity_right * sin(angles_data.heading);
    velocity_data.velocity_east = velocity_data.velocity_forward * sin(angles_data.heading) 
        + velocity_data.velocity_right * cos(angles_data.heading);
    velocity_pub.publish(velocity_data);
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