#include <iostream>
#include <fstream>

#include "camera/camera.h"

using namespace std;

const string Camera::NODE_NAME = "camera";

Camera::Camera()
{
    print_header();
}

void Camera::init_connection(ipc::Communicator& comm)
{
    /**
        Регистрация всех исходящих сообщений навига
    */
    frame_pub_ = comm.advertise<camera::MsgCameraFrame>();

    comm.subscribe_cmd<Camera, camera::CmdCameraConfig>(&Camera::handle_message, this);
}

void Camera::print_header()
{
}

void Camera::print_sensors_info()
{

}

void Camera::handle_message(const camera::CmdCameraConfig& msg)
{
    cout << "new data CmdCameraConfig" << endl; 
}

void Camera::publish_frame(const ros::TimerEvent& event)
{
    camera::MsgCameraFrame msg;
    
    msg.camera_type = 0;
    msg.width       = 400;
    msg.height      = 300;
    msg.channels    = 3;
    msg.frame.assign (127, msg.width * msg.height);

    cout << "send MsgCameraFrame data" << endl;
    frame_pub_.publish(msg);
}