/**
\file
\brief Реализация драйвера камеры

В данном файле находятся реализации методов, объявленных в camera.h

\ingroup camera_node
*/

///@{

#include <iostream>
#include <fstream>

#include "camera/camera.h"

using namespace std;

const string Camera::NODE_NAME = "camera";

Camera::Camera()
{
    print_header();
    ball_taken = false;
}

void Camera::init_connection(ipc::Communicator& comm)
{
    /**
        Регистрация всех исходящих сообщений навига
    */
    frame_pub_ = comm.advertise<camera::MsgCameraFrame>();

    comm.subscribe_cmd<Camera, camera::CmdCameraConfig>(&Camera::handle_message, this);
    comm.subscribe<Camera, supervisor::MsgSupervisorBall>("supervisor", &Camera::handle_message, this);
}

void Camera::print_header()
{
}

void Camera::print_sensors_info()
{

}

void Camera::handle_message(const camera::CmdCameraConfig& msg)
{
    ROS_DEBUG_STREAM("Received "<< ipc::classname(msg) << " msg");
}

void Camera::handle_message(const supervisor::MsgSupervisorBall& msg)
{
    ROS_DEBUG_STREAM("Received "<< ipc::classname(msg) << " msg");
    ball_taken = true;
}

void Camera::publish_frame(const ros::TimerEvent& event)
{
    camera::MsgCameraFrame msg;
    
    msg.camera_type = 0;
    msg.width       = 400;
    msg.height      = 300;
    msg.channels    = 3;
    msg.frame.assign (127, msg.width * msg.height);

    ROS_DEBUG_STREAM_COND(ball_taken, "Publish "<< ipc::classname(msg) << " msg");
    if(ball_taken) {
        frame_pub_.publish(msg);
        ball_taken = false;
    }
}

///@}