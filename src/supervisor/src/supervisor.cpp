/**
\file
\brief Реализация драйвера супервизора

В данном файле находятся реализации методов, объявленных в supervisor.h

\ingroup supervisor_node
*/

///@{

#include <iostream>
#include <fstream>
#include <cstring>
#include <unistd.h>
#include <iomanip>

#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>

#include "supervisor/supervisor.h"

using namespace std;

const string Supervisor::NODE_NAME = "supervisor";

Supervisor::Supervisor()
{
    print_header();
    ball_taken = true;
    publish_time = ros::Time::now().toSec();
}

void Supervisor::init_connection(ipc::Communicator& comm)
{
    /**
        Регистрация всех исходящих сообщений навига
    */
    leak_pub_           = comm.advertise<supervisor::MsgSupervisorLeak>();
    compensator_pub_    = comm.advertise<supervisor::MsgSupervisorCompensator>();
    devices_status_pub_ = comm.advertise<supervisor::MsgSupervisorDevicesStatus>();
    short_circuit_pub_  = comm.advertise<supervisor::MsgSupervisorShortCircuit>();
    adc_pub_            = comm.advertise<supervisor::MsgSupervisorAdc>();
    external_adc_pub_   = comm.advertise<supervisor::MsgSupervisorExternalAdc>();
    depth_pub_          = comm.advertise<supervisor::MsgSupervisorDepth>();
    ball_pub_           = comm.advertise<supervisor::MsgSupervisorBall>();

    /**
        Подписка на сообщения
    */

    comm.subscribe_cmd<Supervisor, supervisor::CmdSupervisorCan>(&Supervisor::handle_message, this);
    comm.subscribe_cmd<Supervisor, supervisor::CmdSupervisorConfigureUdp>(&Supervisor::handle_message, this);
    comm.subscribe_cmd<Supervisor, supervisor::CmdSupervisorDeviceKey>(&Supervisor::handle_message, this);
    comm.subscribe_cmd<Supervisor, supervisor::CmdSupervisorFirmware>(&Supervisor::handle_message, this);
    comm.subscribe_cmd<Supervisor, supervisor::CmdSupervisorPwm>(&Supervisor::handle_message, this);
    comm.subscribe_cmd<Supervisor, supervisor::CmdSupervisorSystemFlags>(&Supervisor::handle_message, this);
    comm.subscribe_cmd<Supervisor, supervisor::CmdSupervisorUart>(&Supervisor::handle_message, this);
}

void Supervisor::print_header()
{
}

void Supervisor::print_sensors_info()
{

}

void Supervisor::handle_message(const supervisor::CmdSupervisorCan& msg)
{
    ROS_DEBUG_STREAM("Receive " << ipc::classname(msg) << " msg");
    supervisor::MsgSupervisorBall ball_msg;
    ball_pub_.publish(ball_msg);
    ROS_DEBUG_STREAM("Publish " << ipc::classname(ball_msg) << " msg");
    publish_time = ros::Time::now().toSec();
}
void Supervisor::handle_message(const supervisor::CmdSupervisorConfigureUdp& msg)
{
    ROS_DEBUG_STREAM("Receive "<< ipc::classname(msg) << " msg");
}
void Supervisor::handle_message(const supervisor::CmdSupervisorDeviceKey& msg)
{
    ROS_DEBUG_STREAM("Receive "<< ipc::classname(msg) << " msg");
}
void Supervisor::handle_message(const supervisor::CmdSupervisorFirmware& msg)
{
    ROS_DEBUG_STREAM("Receive "<< ipc::classname(msg) << " msg");
}
void Supervisor::handle_message(const supervisor::CmdSupervisorPwm& msg)
{
    ROS_DEBUG_STREAM("Receive "<< ipc::classname(msg) << " msg");
}
void Supervisor::handle_message(const supervisor::CmdSupervisorSystemFlags& msg)
{
    ROS_DEBUG_STREAM("Receive "<< ipc::classname(msg) << " msg");
}
void Supervisor::handle_message(const supervisor::CmdSupervisorUart& msg)
{
    ROS_DEBUG_STREAM("Receive "<< ipc::classname(msg) << " msg");
}

void Supervisor::publish_leak(const ros::TimerEvent& event)
{
    supervisor::MsgSupervisorLeak msg;
    vector<int> model_array = {1,0,1,0,1,0,1,0};
    copy(model_array.begin(), model_array.end(), msg.status.begin());
    leak_pub_.publish(msg);
    // ROS_DEBUG_STREAM("Publish "<< ipc::classname(msg) << " msg");
}

void Supervisor::publish_compensator(const ros::TimerEvent& event)
{
    supervisor::MsgSupervisorCompensator msg;
    vector<int> model_array = {1,0,1,0,1,0,1,0};
    copy(model_array.begin(), model_array.end(), msg.status.begin());
    compensator_pub_.publish(msg);
    // ROS_DEBUG_STREAM("Publish "<< ipc::classname(msg) << " msg");
}

void Supervisor::publish_devices_status(const ros::TimerEvent& event)
{
    supervisor::MsgSupervisorDevicesStatus msg;
    vector<int> model_array = {1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0};
    copy(model_array.begin(), model_array.end(), msg.status.begin());
    devices_status_pub_.publish(msg);
    // ROS_DEBUG_STREAM("Publish "<< ipc::classname(msg) << " msg");
}

void Supervisor::publish_short_circuit(const ros::TimerEvent& event)
{
    supervisor::MsgSupervisorShortCircuit msg;
    vector<int> model_array = {1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0};
    copy(model_array.begin(), model_array.end(), msg.status.begin());
    short_circuit_pub_.publish(msg);
    // ROS_DEBUG_STREAM("Publish "<< ipc::classname(msg) << " msg");
}

void Supervisor::publish_adc(const ros::TimerEvent& event)
{
    supervisor::MsgSupervisorAdc msg;
    vector<float> model_array;
    model_array.assign (11, 1.17);
    copy(model_array.begin(), model_array.end(), msg.values.begin());
    adc_pub_.publish(msg);
    // ROS_DEBUG_STREAM("Publish "<< ipc::classname(msg) << " msg");
}

void Supervisor::publish_external_adc(const ros::TimerEvent& event)
{
    supervisor::MsgSupervisorExternalAdc msg;
    vector<float> model_array;
    model_array.assign (6, 1.78);
    copy(model_array.begin(), model_array.end(), msg.values.begin());
    external_adc_pub_.publish(msg);
    // ROS_DEBUG_STREAM("Publish "<< ipc::classname(msg) << " msg");
}

void Supervisor::publish_depth(const ros::TimerEvent& event)
{
    supervisor::MsgSupervisorDepth msg;
    msg.depth = 2.6;
    depth_pub_.publish(msg);
    ROS_DEBUG_STREAM("Publish "<< ipc::classname(msg) << " msg");
}

void Supervisor::publish_ball(const ros::TimerEvent& event)
{
    if(ros::Time::now().toSec() - publish_time > 1.0) {
        supervisor::MsgSupervisorBall msg;
        ball_pub_.publish(msg);
        ROS_DEBUG_STREAM("Ball_taken publish by timeout");
        publish_time = ros::Time::now().toSec();
    }
}
///@}