#pragma once

#include <string>

#include <ros/ros.h>

#include <libipc/ipc.h>

class Navig
{
public:
    Navig();
    Navig(const Navig& lhs);
    virtual ~Navig();

    static const std::string NODE_NAME;

    void init_ipc(int argc, char* argv[], const std::string& node_name);

    void create_and_publish_acc();
    void create_and_publish_angles();
    void create_and_publish_depth();
    void create_and_publish_height();
    void create_and_publish_position();
    void create_and_publish_rates();
    void create_and_publish_velocity();

private:
    std::pair<double, double> local_position_;
    double longitude_, latitude_;
    float acc_x_, acc_y_, acc_z_;
    float heading_, pitch_, roll_;
    float depth_;
    float distance_forward_, distance_backward_, distance_rightward_, distance_leftward_;
    float height_;
    float rate_heading_, rate_roll_, rate_pitch_;
    float velocity_forward_, velocity_right_, velocity_down_;

    ros::Publisher acc_pub_, angles_pub_, depth_pub_, height_pub_, position_pub_, rates_pub_, 
        velocity_pub_;
};
