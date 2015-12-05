#include <iostream>
#include <fstream>

#include "echometer/echometer.h"

using namespace std;

const string Echometer::NODE_NAME = "Echometer";

Echometer::Echometer()
{
    print_header();
}

void Echometer::init_connection(ipc::Communicator& comm)
{
    /**
        Регистрация всех исходящих сообщений навига
    */
    height_pub_       = comm.advertise<echometer::MsgEchometerHeight>();
    temperature_pub_  = comm.advertise<echometer::MsgEchometerTemperature>();
}

void Echometer::print_header()
{
}

void Echometer::print_sensors_info()
{

}

void Echometer::publish_height(const ros::TimerEvent& event)
{
    echometer::MsgEchometerHeight msg;
    msg.height = 25.3;
    cout << "send MsgEchometerHeight data" << endl;
    height_pub_.publish(msg);
}

void Echometer::publish_temperature(const ros::TimerEvent& event)
{
    echometer::MsgEchometerTemperature msg;
    msg.temperature = 36.6;
    cout << "send MsgEchometerTemperature data" << endl;
    temperature_pub_.publish(msg);
}