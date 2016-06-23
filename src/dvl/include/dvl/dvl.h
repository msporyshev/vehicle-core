/**
\file
\brief Заголовочный файл драйвера доплера

В данном файле находся объявление класса драйвера доплера

\ingroup dvl_node
*/

///@{

#pragma once

#include <string>
#include <yaml_reader.h>

#include <libipc/ipc.h>
#include "ros/ros.h"

#include "dvl/MsgDownwardDistance.h"
#include "dvl/MsgDownwardVelocity.h"
#include "dvl/MsgPlaneVelocity.h"

#include "dvl/DVL_TRDI.h"

struct dvlConfig
{
    std::string port;
    int         baudrate;
    bool        modelling;
    bool        start_now;

    dvlConfig() {
        port = "";
        baudrate = 0;
        start_now = false;
        modelling = false;
    }
};

struct dvlDistance
{
    bool is_new;
    float forward;
    float rightward;
    float downward;
    dvlDistance() {
        is_new = false;
        forward    = 0;
        rightward  = 0;
        downward   = 0;
    }
};

struct dvlVelocity
{
    bool is_new;
    float downward;
    float forward;
    float rightward;
    dvlVelocity() {
        is_new = false;
        downward  = 0;
        forward   = 0;
        rightward = 0;
    }
};

class Dvl
{

    void print_header();
    void print_sensors_info();

public:
    Dvl(dvlConfig config);
    const static std::string NODE_NAME;

    void publish_distance(const ros::TimerEvent& event);
    void publish_velocity(const ros::TimerEvent& event);

    void init_dvl();
    void deinit_dvl();
    void init_connection(ipc::Communicator& comm);
    void start_timers(ipc::Communicator& comm);

    void data_update(const ros::TimerEvent& event);
    void data_update_modelling(const ros::TimerEvent& event);

private:
    ros::Publisher  downward_distance_pub_,
                    downward_velocity_pub_,
                    plane_velocity_pub_;
    
    dvlConfig config_;
    bool new_data_avalible_;

    dvlDistance     distance_;
    dvlVelocity     velocity_;

    DvlTrdiDriver dvl_trdi_;

    ros::Timer timer_data_update_;
    ros::Timer timer_pub_distance_;
    ros::Timer timer_pub_velocity_;
};
///@}