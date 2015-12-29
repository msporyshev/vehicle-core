/**
\file
\brief Реализация драйвера GPS

В данном файле находятся реализации методов, объявленных в gps.h

\ingroup gps_node
*/

///@{

#include <iostream>
#include <fstream>

#include "gps.h"

using namespace std;

const string Gps::NODE_NAME = "gps";

Gps::Gps()
{
    print_header();
}

void Gps::init_connection(ipc::Communicator& comm)
{
    /**
        Регистрация всех исходящих сообщений навига
    */
    coordinate_pub_    = comm.advertise<gps::MsgGpsCoordinate>();
    satellites_pub_    = comm.advertise<gps::MsgGpsSatellites>();
    utc_pub_           = comm.advertise<gps::MsgGpsUtc>();
}

void Gps::print_header()
{
}

void Gps::print_sensors_info()
{

}

void Gps::publish_coordinate(const ros::TimerEvent& event)
{
    gps::MsgGpsCoordinate msg;
    msg.lat = 135.04543;
    msg.lon = 54.04543;
    ROS_INFO_STREAM("Published " << ipc::classname(msg));
    coordinate_pub_.publish(msg);
}

void Gps::publish_satellites(const ros::TimerEvent& event)
{
    gps::MsgGpsSatellites msg;
    msg.satellites = 8;
    ROS_INFO_STREAM("Published " << ipc::classname(msg));
    satellites_pub_.publish(msg);
}

void Gps::publish_utc(const ros::TimerEvent& event)
{
    gps::MsgGpsUtc msg;
    msg.utc = 1234321.098;
    ROS_INFO_STREAM("Published " << ipc::classname(msg));
    utc_pub_.publish(msg);
}

///@}