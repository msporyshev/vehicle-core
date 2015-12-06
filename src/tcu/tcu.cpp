/**
\file
\brief Реализация модуля управления движителями

В данном файле находятся реализации методов, объявленных в tcu.h
Также здесь находится main
*/
#include <vector>

#include <supervisor/CmdSupervisorCan.h>

#include "tcu.h"

const std::string Tcu::NODE_NAME = "tcu";

Tcu::Tcu(ipc::Communicator& communicator) :
    communicator_(communicator) 
{
    this->init_ipc();
}

Tcu::~Tcu()
{}

void Tcu::init_ipc()
{
	this->can_send_pub_ = this->communicator_.advertise<supervisor::CmdSupervisorCan>(); 

	communicator_.subscribe("motion", &Tcu::process_and_publish_regul, this);
}

void Tcu::create_and_publish_can_send()
{
	supervisor::CmdSupervisorCan msg;
		
	msg.can_id = 20;

	std::vector<int> model_array = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
    copy(model_array.begin(), model_array.end(), msg.can_data.begin());

    ROS_INFO("I sent command to id %d", msg.can_id);

    can_send_pub_.publish(msg);
}

void Tcu::process_and_publish_regul(const motion::MsgRegul& msg)
{
    supervisor::CmdSupervisorCan nmsg;
    nmsg.can_id = 20;

    std::vector<int8_t> model_array = {(int8_t)msg.tx, (int8_t)msg.ty, (int8_t)msg.tz, (int8_t)msg.mx, (int8_t)msg.my, (int8_t)msg.mz, 0x07, 0x08};
    copy(model_array.begin(), model_array.end(), nmsg.can_data.begin());

    ROS_INFO("I received and sent command witx tx = %f to id %d", msg.tx, nmsg.can_id);

    can_send_pub_.publish(nmsg);
}

int main(int argc, char **argv)
{
	auto communicator = ipc::init(argc, argv, Tcu::NODE_NAME);
    Tcu tcu(communicator);

    ipc::EventLoop loop(5);
    while(loop.ok()) 
    {
        tcu.create_and_publish_can_send();        
    }
    
    return 0;

}