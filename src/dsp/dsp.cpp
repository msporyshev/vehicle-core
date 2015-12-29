/**
\file
\brief Реализация модуля определения координат акустических маяков

В данном файле находятся реализации методов, объявленных в dsp.h
Также здесь находится main

\defgroup dsp_node DSP
\brief Данный нод предназначен для инициализации соединения с DSP, 
корректного выключения, а так же получения данных от DSP и их публикации в сеть.
*/

///@{
#include <vector>

#include <dsp/MsgDspBeacon.h>

#include "dsp.h"

const std::string Dsp::NODE_NAME = "dsp";

Dsp::Dsp(ipc::Communicator& communicator) :
    communicator_(communicator) 
{
    this->init_ipc();
}

Dsp::~Dsp()
{}

void Dsp::init_ipc()
{
	this->beacon_pub_ = this->communicator_.advertise<dsp::MsgDspBeacon>(); 

	communicator_.subscribe("navig", &Dsp::handle_depth, this);
}

void Dsp::create_and_publish_beacon()
{
	dsp::MsgDspBeacon msg;
		
	msg.x = 2.0;
    msg.y = 2.0;
    msg.z = 2.0;

    msg.theta = 1.0;
    msg.phi = 1.0;
    msg.dist = 12.0;

    msg.freq_khz = 37.5;    

    ROS_INFO_STREAM("Published " << ipc::classname(msg));
    // ROS_INFO("I sent x: %f, y: %f, z: %f for beacon with frequency %fkhz", msg.x, msg.y, msg.z, msg.freq_khz);

    beacon_pub_.publish(msg);
}

void Dsp::handle_depth(const navig::MsgNavigDepth& msg)
{
    ROS_INFO_STREAM("Received " << ipc::classname(msg));
    // ROS_INFO("I received depth %f", msg.depth);
}

int main(int argc, char **argv)
{
	auto communicator = ipc::init(argc, argv, Dsp::NODE_NAME);
    Dsp dsp(communicator);

    ipc::EventLoop loop(1);
    while(loop.ok()) 
    {
        dsp.create_and_publish_beacon();        
    }
    
    return 0;

}

///@}