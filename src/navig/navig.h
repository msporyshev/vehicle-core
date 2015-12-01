#pragma once

#include <string>

#include <ros/ros.h>

#include <compass/MsgCompassAngle.h>
#include <compass/MsgCompassAcceleration.h>
#include <compass/MsgCompassAngleRate.h>
#include <dvl/MsgDvlDistanceBackward.h>
#include <dvl/MsgDvlDistanceForward.h>
#include <dvl/MsgDvlDistanceLeftward.h>
#include <dvl/MsgDvlDistanceRightward.h>
#include <dvl/MsgDvlVelocityDown.h>
#include <dvl/MsgDvlVelocityForward.h>
#include <dvl/MsgDvlVelocityRight.h>
#include <dvl/MsgDvlHeight.h>
#include <gps/MsgGpsCoordinate.h>
#include <gps/MsgGpsSatellites.h>
#include <gps/MsgGpsUtc.h>
#include <sucan/MsgSucanDepth.h>

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

    template<typename T>
    void handle_message(const T& msg)
    {
        std::cout << "Message " << ros::message_traits::datatype<T>() << " received" << std::endl;
        std::cout << msg << std::endl;
    }
    
    // void handle_angles(const compass::msgCompassAngle& msg);
    // void handle_acceleration(const compass::msgCompassAcceleration& msg);
    // void handle_rate(const compass::msgCompassAngleRate& msg);
    // void handle_distance_backward(const dvl::MsgDvlDistanceBackward& msg);
    // void handle_distance_forward(const dvl::MsgDvlDistanceForward& msg);
    // void handle_distance_leftward(const dvl::MsgDvlDistanceLeftward& msg);
    // void handle_distance_rightward(const dvl::MsgDvlDistanceRightward& msg);
    // void handle_velocity_down(const dvl::MsgDvlVelocityDown& msg);
    // void handle_velocity_forward(const dvl::MsgDvlVelocityForward& msg);
    // void handle_velocity_right(const dvl::MsgDvlVelocityRight& msg);
    // void handle_height(const dvl::MsgDvlHeight& msg);
    // void handle_coordinate(const gps::MsgGpsCoordinate& msg);
    // void handle_satellites(const gps::MsgGpsSatellites& msg);
    // void handle_utc(const gps::MsgGpsUtc& msg);
    // void handle_depth(const sucan::MsgSucanDepth& msg);

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
