#include <iostream>
#include <fstream>

#include "sidesonar/sidesonar.h"

using namespace std;

const string Sidesonar::NODE_NAME = "Sidesonar";

Sidesonar::Sidesonar()
{
    print_header();
}

void Sidesonar::init_connection(ipc::Communicator& comm)
{
    /**
        Регистрация всех исходящих сообщений навига
    */
    line_pub_ = comm.advertise<sidesonar::MsgSidesonarLine>();

    comm.subscribe_cmd<Sidesonar, sidesonar::CmdSidesonarConfig>(&Sidesonar::handle_message, this);
    comm.subscribe_cmd<Sidesonar, sidesonar::CmdSidesonarWorkStatus>(&Sidesonar::handle_message, this);

}

void Sidesonar::print_header()
{
}

void Sidesonar::print_sensors_info()
{

}

void Sidesonar::handle_message(const sidesonar::CmdSidesonarConfig& msg)
{
    cout << "new data CmdSidesonarConfig" << endl; 
}

void Sidesonar::handle_message(const sidesonar::CmdSidesonarWorkStatus& msg)
{
    cout << "new data CmdSidesonarWorkStatus" << endl; 
}

void Sidesonar::publish_line(const ros::TimerEvent& event)
{
    sidesonar::MsgSidesonarLine msg;
    msg.size = 400;
    vector<int> model_array;
    model_array.assign (199, msg.size);

    msg.data_line.resize(model_array.size());
    copy(model_array.begin(), model_array.end(), msg.data_line.begin());
    
    cout << "send MsgSidesonarHeight data" << endl;
    line_pub_.publish(msg);
}