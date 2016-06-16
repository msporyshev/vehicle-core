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

#include "dvl/MsgHeight.h"
#include "dvl/MsgDistance.h"
#include "dvl/MsgVelocity.h"

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

struct dvlHeigth
{
    bool is_new;
    float heigth;
    dvlHeigth() {
        is_new = false;
        heigth = 0;
    }
};

struct dvlDistance
{
    bool is_new;
    float backward;
    float forward;
    float leftward;
    float rightward;
    dvlDistance() {
        is_new = false;
        backward  = 0;
        forward   = 0;
        leftward  = 0;
        rightward = 0;
    }
};

struct dvlVelocity
{
    bool is_new;
    float down;
    float forward;
    float right;
    dvlVelocity() {
        is_new = false;
        down    = 0;
        forward = 0;
        right   = 0;
    }
};

class Dvl
{

    void print_header();
    void print_sensors_info();

public:
    Dvl(dvlConfig config);
    const static std::string NODE_NAME;

    void publish_height(const ros::TimerEvent& event);
    void publish_distance(const ros::TimerEvent& event);
    void publish_velocity(const ros::TimerEvent& event);

    void init_dvl();
    void deinit_dvl();
    void init_connection(ipc::Communicator& comm);
    void start_timers(ipc::Communicator& comm);

    void data_update(const ros::TimerEvent& event);
    void data_update_modelling(const ros::TimerEvent& event);

private:
    ros::Publisher  height_pub_,
                    distance_pub_,
                    velocity_pub_;
    
    dvlConfig config_;
    bool new_data_avalible_;

    dvlHeigth       heigth_;
    dvlDistance     distance_;
    dvlVelocity     velocity_;

    DvlTrdiDriver dvl_trdi_;

    ros::Timer timer_data_update_;
    ros::Timer timer_data_publish_heigth_;
    ros::Timer timer_data_publish_distance_;
    ros::Timer timer_data_publish_velocity_;
};
///@}