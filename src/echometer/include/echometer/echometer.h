/**
\file
\brief Заголовочный файл драйвера эхолота

В данном файле находится обьявление класса драйвера эхолота

\ingroup echometer_node
*/

///@{

#pragma once

#include <string>
#include <yaml_reader.h>

#include <libipc/ipc.h>
#include "ros/ros.h"

#include "echometer/MsgHeight.h"
#include "echometer/MsgTemperature.h"

#include "echometer/CmdConfig.h"
#include "echometer/CmdWorkStatus.h"

#include "echometer/driver_echometer.h"

typedef struct {
    float data;
    double time;
    bool is_new;
} device_data_struct;

class Echometer
{

    void print_header();
    void print_sensors_info();

public:
    Echometer();
    const static std::string NODE_NAME;

    void update_data(const ros::TimerEvent& event);
    void publish_height(const ros::TimerEvent& event);
    void publish_temperature(const ros::TimerEvent& event);
    void publish_simulate_height(const ros::TimerEvent& event);
    void publish_simulate_temperature(const ros::TimerEvent& event);

    void handle_message(const echometer::CmdConfig& msg);
    void handle_message(const echometer::CmdWorkStatus& msg);

    void init_connection(ipc::Communicator& communicator);

    void read_config();
    void connect_device();
    void disconnect_device();
    bool get_simulate_status();
    bool get_instant_start_status();

private:
    ros::Publisher  height_pub_,
                    temperature_pub_;
    bool simulate_status_ = 0;
    bool instant_start_ = 0;
    bool connect_state_ = 0;
    echosouder_config_struct echosouder_config_;

    device_data_struct height_data_;
    device_data_struct temperature_data_;
};

///@}