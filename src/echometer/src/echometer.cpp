/**
\file
\brief Реализация драйвера эхолота

В данном файле находятся реализации методов, объявленных в echometer.h

\ingroup echometer_node
*/

///@{

#include <iostream>
#include <fstream>

#include "echometer/echometer.h"

using namespace std;

const string Echometer::NODE_NAME = "echometer";


Echometer::Echometer()
{
    print_header();

    height_data_.data   = 0;
    height_data_.time   = 0;
    height_data_.is_new = 0;

    temperature_data_.data   = 0;
    temperature_data_.time   = 0;
    temperature_data_.is_new = 0;
}

void Echometer::init_connection(ipc::Communicator& comm)
{
    height_pub_       = comm.advertise<echometer::MsgHeight>();
    temperature_pub_  = comm.advertise<echometer::MsgTemperature>();

    comm.subscribe_cmd<Echometer, echometer::CmdConfig>(&Echometer::handle_message, this);
    comm.subscribe_cmd<Echometer, echometer::CmdWorkStatus>(&Echometer::handle_message, this);
}

void Echometer::print_header()
{
}

void Echometer::print_sensors_info()
{

}

void Echometer::read_config()
{
    int temp;
    ROS_ASSERT(ros::param::get("/echometer/device_config/workmode", temp));
    echosouder_config_.workm = static_cast<Workmode>(temp);
    ROS_ASSERT(ros::param::get("/echometer/device_config/range", echosouder_config_.range));
    ROS_ASSERT(ros::param::get("/echometer/device_config/interval", echosouder_config_.interval));
    ROS_ASSERT(ros::param::get("/echometer/device_config/threshold", echosouder_config_.threshold));
    ROS_ASSERT(ros::param::get("/echometer/device_config/offset", echosouder_config_.offset));
    ROS_ASSERT(ros::param::get("/echometer/device_config/deadzone", echosouder_config_.deadzone));
    ROS_ASSERT(ros::param::get("/echometer/device_config/syncextern", temp));
    echosouder_config_.syncextern = static_cast<Syncmode>(temp);
    ROS_ASSERT(ros::param::get("/echometer/device_config/syncextmod", temp));
    echosouder_config_.syncextmod = static_cast<Syncedge>(temp);
    ROS_ASSERT(ros::param::get("/echometer/work_config/simulate", temp));
    simulate_status_ = static_cast<bool>(temp);
    ROS_ASSERT(ros::param::get("/echometer/work_config/instant_start", temp));
    instant_start_ = static_cast<bool>(temp);
}

void Echometer::connect_device()
{
    if(connect_state_ == 1) {
        ROS_WARN_STREAM("Echometer is already connected");
    }

    string com_name;
    int com_baudrate;

    ROS_ASSERT(ros::param::get("/echometer/connection_config/com_port", com_name));
    ROS_ASSERT(ros::param::get("/echometer/connection_config/com_baudrate", com_baudrate));

    bool com_status = open_echosounder(com_name.c_str(), com_baudrate);
    ROS_ASSERT_MSG(!com_status, "FAIL: Can not open COM port");

    ros::Duration(POWER_ON_WAIT / 1000).sleep();

    echosounder_set_config(echosouder_config_);

    connect_state_ == 1;
}

void Echometer::disconnect_device()
{
    if(connect_state_ == 0) {
        ROS_WARN_STREAM("Echometer is already disconnected");
    }

    close_echosounder();

    ros::Duration(POWER_ON_WAIT / 1000).sleep();

    connect_state_ == 0;
}

bool Echometer::get_simulate_status()
{
    return simulate_status_;
}

bool Echometer::get_instant_start_status()
{
    return instant_start_;
}

void Echometer::handle_message(const echometer::CmdConfig& msg)
{
    if(connect_state_ == 1) {
        echosouder_config_.workm       = static_cast<Workmode>(msg.workm);
        echosouder_config_.range       = msg.range;
        echosouder_config_.interval    = msg.threshold;
        echosouder_config_.threshold   = msg.interval;
        echosouder_config_.offset      = msg.offset;
        echosouder_config_.deadzone    = msg.deadzone;
        echosouder_config_.syncextern  = static_cast<Syncmode>(msg.syncextern);
        echosouder_config_.syncextmod  = static_cast<Syncedge>(msg.syncextmod);
    } else {
        ROS_WARN_STREAM("Echometer is disconnected");
    }
}

void Echometer::handle_message(const echometer::CmdWorkStatus& msg)
{
    if(msg.status == 0) {
        disconnect_device();
    } else {
        connect_device();
    }
}

void Echometer::update_data(const ros::TimerEvent& event)
{
    if(!connect_state_) {
        return;
    }

    echosouder_data_struct data;

    bool is_new = get_data(data);

    if(is_new) {

        if(!data.bad_data) {
            height_data_.data   = data.depth_below;
            height_data_.time   = data.time;
            height_data_.is_new = true;
        } else {
            ROS_DEBUG_STREAM("Received bad data about height:" << height_data_.data);
        }

        temperature_data_.data   = data.temperature;
        temperature_data_.time   = data.time;
        temperature_data_.is_new = true;
    }
}

void Echometer::publish_height(const ros::TimerEvent& event)
{
    if(!height_data_.is_new) {
        return;
    }

    echometer::MsgHeight msg;
    msg.height = height_data_.data;
    ROS_DEBUG_STREAM("Published " << ipc::classname(msg));
    height_pub_.publish(msg);
}

void Echometer::publish_temperature(const ros::TimerEvent& event)
{
    if(!temperature_data_.is_new) {
        return;
    }

    echometer::MsgTemperature msg;
    msg.temperature = temperature_data_.data;
    ROS_DEBUG_STREAM("Published " << ipc::classname(msg));
    temperature_pub_.publish(msg);
}

void Echometer::publish_simulate_height(const ros::TimerEvent& event)
{
    static float height = 0;
    height = height > 100 ? 0 : height + 0.3;

    echometer::MsgHeight msg;
    msg.height = height;
    ROS_DEBUG_STREAM("Published " << ipc::classname(msg));
    height_pub_.publish(msg);
}

void Echometer::publish_simulate_temperature(const ros::TimerEvent& event)
{
    static float temperature = 0;
    temperature = temperature > 40 ? 0 : temperature + 0.2;

    echometer::MsgTemperature msg;
    msg.temperature = temperature;
    ROS_DEBUG_STREAM("Published " << ipc::classname(msg));
    temperature_pub_.publish(msg);
}

///@}
