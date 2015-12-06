#pragma once

#include <string>
#include <yaml_reader.h>

#include <libipc/ipc.h>
#include "ros/ros.h"

#include "dvl/MsgDvlHeight.h"
#include "dvl/MsgDvlDistance.h"
#include "dvl/MsgDvlVelocity.h"

class Dvl
{

    void print_header();
    void print_sensors_info();

public:
    Dvl();
    const static std::string NODE_NAME;

    void publish_height(const ros::TimerEvent& event);
    void publish_distance(const ros::TimerEvent& event);
    void publish_velocity(const ros::TimerEvent& event);

    void init_connection(ipc::Communicator& communicator);

private:
    ros::Publisher  height_pub_,
                    distance_pub_,
                    velocity_pub_;
};