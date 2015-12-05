#pragma once

#include <string>
#include <yaml_reader.h>

#include <libipc/ipc.h>
#include "ros/ros.h"

#include "echometer/MsgEchometerHeight.h"
#include "echometer/MsgEchometerTemperature.h"

class Echometer
{

    void print_header();
    void print_sensors_info();

public:
    Echometer();
    const static std::string NODE_NAME;

    void publish_height(const ros::TimerEvent& event);
    void publish_temperature(const ros::TimerEvent& event);

    void init_connection(ipc::Communicator& communicator);

private:
    ros::Publisher  height_pub_,
                    temperature_pub_;
};