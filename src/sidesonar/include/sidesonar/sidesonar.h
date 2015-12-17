/**
\file
\brief Заголовочный файл драйвера ГБО

В данном файле находится обьявление класса драйвера ГБО

\ingroup sidesonar_node
*/

///@{


#pragma once

#include <string>
#include <yaml_reader.h>

#include <libipc/ipc.h>
#include "ros/ros.h"

#include "sidesonar/MsgSidesonarLine.h"

#include "sidesonar/CmdSidesonarConfig.h"
#include "sidesonar/CmdSidesonarWorkStatus.h"

class Sidesonar
{

    void print_header();
    void print_sensors_info();

public:
    Sidesonar();
    const static std::string NODE_NAME;

    void publish_line(const ros::TimerEvent& event);

    void init_connection(ipc::Communicator& communicator);

private:

	void handle_message(const sidesonar::CmdSidesonarConfig& msg);
	void handle_message(const sidesonar::CmdSidesonarWorkStatus& msg);


    ros::Publisher  line_pub_;
};

///@}