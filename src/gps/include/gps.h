#pragma once

#include <string>
#include <yaml_reader.h>

#include <libipc/ipc.h>
#include "ros/ros.h"

#include "gps/MsgGpsCoordinate.h"
#include "gps/MsgGpsSatellites.h"
#include "gps/MsgGpsUtc.h"

class Gps
{

    void print_header();
    void print_sensors_info();

public:
    Gps();
    const static std::string NODE_NAME;

    void publish_coordinate(const ros::TimerEvent& event);
    void publish_satellites(const ros::TimerEvent& event);
    void publish_utc(const ros::TimerEvent& event);

    void init_connection(ipc::Communicator& communicator);

private:
    ros::Publisher  coordinate_pub_,
                    satellites_pub_,
                    utc_pub_;
};