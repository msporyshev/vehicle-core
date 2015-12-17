/**
\file
\brief Заголовочный файл драйвера камеры

В данном файле находится обьявление класса драйвера камеры

\ingroup camera_node
*/

///@{

#pragma once

#include <string>
#include <yaml_reader.h>

#include <libipc/ipc.h>
#include "ros/ros.h"

#include "camera/MsgCameraFrame.h"
#include "camera/CmdCameraConfig.h"

class Camera
{

    void print_header();
    void print_sensors_info();

public:
    Camera();
    const static std::string NODE_NAME;

    void publish_frame(const ros::TimerEvent& event);

    void init_connection(ipc::Communicator& communicator);

private:
	void handle_message(const camera::CmdCameraConfig& msg);

    ros::Publisher  frame_pub_;
};

///@}