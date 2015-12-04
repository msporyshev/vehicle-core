#pragma once

#include <string>
#include <iostream>

#include <ros/ros.h>

#include <libipc/ipc.h>

class MotionServer
{
public:
    MotionServer(ipc::Communicator& communicator);
    ~MotionServer();

    void init_ipc();

    void create_and_publish_cmd_status();
    void create_and_publish_regul();

    template<typename T>
    void handle_command(const T& msg)
    {
        std::cout << "Message " << ros::message_traits::datatype<T>() << " received" << std::endl;
        std::cout << msg << std::endl;
    }

    static const std::string NODE_NAME;

private:
    ipc::Communicator& communicator_;
    ros::Publisher cmd_status_pub_, regul_pub_;
};
