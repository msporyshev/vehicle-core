#pragma once

#include <string>

#include <ros/ros.h>

class MotionServer
{
public:
    MotionServer();
    ~MotionServer();

    void init_ipc(int argc, char* argv[], const std::string& node_name);

    void create_and_publish_cmd_status();
    void create_and_publish_regul();

    static const std::string NODE_NAME;

private:
    ros::Publisher cmd_status_pub_, regul_pub_;
};