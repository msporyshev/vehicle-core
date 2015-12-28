/**
\file
\brief Заголовочный файл драйвера супервизора

В данном файле находится обьявление класса драйвера супервизора

\ingroup supervisor_node
*/

///@{

#pragma once

#include <string>
#include <yaml_reader.h>

#include <libipc/ipc.h>
#include "ros/ros.h"

#include "supervisor/MsgSupervisorLeak.h"
#include "supervisor/MsgSupervisorCompensator.h"
#include "supervisor/MsgSupervisorDevicesStatus.h"
#include "supervisor/MsgSupervisorShortCircuit.h"
#include "supervisor/MsgSupervisorAdc.h"
#include "supervisor/MsgSupervisorExternalAdc.h"
#include "supervisor/MsgSupervisorDepth.h"
#include "supervisor/MsgSupervisorBall.h"

#include "supervisor/CmdSupervisorCan.h"
#include "supervisor/CmdSupervisorConfigureUdp.h"
#include "supervisor/CmdSupervisorDeviceKey.h"
#include "supervisor/CmdSupervisorFirmware.h"
#include "supervisor/CmdSupervisorPwm.h"
#include "supervisor/CmdSupervisorSystemFlags.h"
#include "supervisor/CmdSupervisorUart.h"

class Supervisor
{

    void print_header();
    void print_sensors_info();

public:
    Supervisor();
    const static std::string NODE_NAME;

    void publish_leak(const ros::TimerEvent& event);
    void publish_compensator(const ros::TimerEvent& event);
    void publish_devices_status(const ros::TimerEvent& event);
    void publish_short_circuit(const ros::TimerEvent& event);
    void publish_adc(const ros::TimerEvent& event);
    void publish_external_adc(const ros::TimerEvent& event);
    void publish_depth(const ros::TimerEvent& event);
    void publish_ball(const ros::TimerEvent& event);

    void init_connection(ipc::Communicator& communicator);

private:

    void handle_message(const supervisor::CmdSupervisorCan& msg);
    void handle_message(const supervisor::CmdSupervisorConfigureUdp& msg);
    void handle_message(const supervisor::CmdSupervisorDeviceKey& msg);
    void handle_message(const supervisor::CmdSupervisorFirmware& msg);
    void handle_message(const supervisor::CmdSupervisorPwm& msg);
    void handle_message(const supervisor::CmdSupervisorSystemFlags& msg);
    void handle_message(const supervisor::CmdSupervisorUart& msg);


    ros::Publisher  leak_pub_,
                    compensator_pub_,
                    devices_status_pub_,
                    short_circuit_pub_,
                    adc_pub_,
                    external_adc_pub_,
                    depth_pub_,
                    ball_pub_;
    bool ball_taken;
    double publish_time;
};

///@}