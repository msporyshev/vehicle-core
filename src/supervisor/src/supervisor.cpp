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
    cout << "new data CmdSupervisorCan" << endl; 
}
void Supervisor::handle_message(const supervisor::CmdSupervisorConfigureUdp& msg)
{
    cout << "new data CmdSupervisorConfigureUdp" << endl; 
}
void Supervisor::handle_message(const supervisor::CmdSupervisorDeviceKey& msg)
{
    cout << "new data CmdSupervisorDeviceKey" << endl; 
}
void Supervisor::handle_message(const supervisor::CmdSupervisorFirmware& msg)
{
    cout << "new data CmdSupervisorFirmware" << endl; 
}
void Supervisor::handle_message(const supervisor::CmdSupervisorPwm& msg)
{
    cout << "new data CmdSupervisorPwm" << endl; 
}
void Supervisor::handle_message(const supervisor::CmdSupervisorSystemFlags& msg)
{
    cout << "new data CmdSupervisorSystemFlags" << endl; 
}
void Supervisor::handle_message(const supervisor::CmdSupervisorUart& msg)
{
    cout << "new data CmdSupervisorUart" << endl; 
}

void Supervisor::publish_leak(const ros::TimerEvent& event)
{
    cout << "send MsgSupervisorLeak data" << endl;
    supervisor::MsgSupervisorLeak msg;
    vector<int> model_array = {1,0,1,0,1,0,1,0};
    copy(model_array.begin(), model_array.end(), msg.status.begin());
    leak_pub_.publish(msg);
}

void Supervisor::publish_compensator(const ros::TimerEvent& event)
{
    supervisor::MsgSupervisorCompensator msg;
    cout << "send MsgSupervisorCompensator data" << endl;
    vector<int> model_array = {1,0,1,0,1,0,1,0};
    copy(model_array.begin(), model_array.end(), msg.status.begin());
    compensator_pub_.publish(msg);
}

void Supervisor::publish_devices_status(const ros::TimerEvent& event)
{
    supervisor::MsgSupervisorDevicesStatus msg;
    cout << "send MsgSupervisorDevicesStatus data" << endl;
    vector<int> model_array = {1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0};
    copy(model_array.begin(), model_array.end(), msg.status.begin());
    devices_status_pub_.publish(msg);
}

void Supervisor::publish_short_circuit(const ros::TimerEvent& event)
{
    supervisor::MsgSupervisorShortCircuit msg;
    cout << "send MsgSupervisorShortCircuit data" << endl;
    vector<int> model_array = {1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0};
    copy(model_array.begin(), model_array.end(), msg.status.begin());
    short_circuit_pub_.publish(msg);
}

void Supervisor::publish_adc(const ros::TimerEvent& event)
{
    supervisor::MsgSupervisorAdc msg;
    cout << "send MsgSupervisorAdc data" << endl;
    vector<float> model_array;
    model_array.assign (11, 1.17);
    copy(model_array.begin(), model_array.end(), msg.values.begin());
    adc_pub_.publish(msg);
}

void Supervisor::publish_external_adc(const ros::TimerEvent& event)
{
    supervisor::MsgSupervisorExternalAdc msg;
    cout << "send MsgSupervisorExternalAdc data" << endl;
    vector<float> model_array;
    model_array.assign (6, 1.78);
    copy(model_array.begin(), model_array.end(), msg.values.begin());
    external_adc_pub_.publish(msg);
}

void Supervisor::publish_depth(const ros::TimerEvent& event)
{
    supervisor::MsgSupervisorDepth msg;
    cout << "send MsgSupervisorDepth data" << endl;
    msg.depth = 2.6;
    depth_pub_.publish(msg);

}