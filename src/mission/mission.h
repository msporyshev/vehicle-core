#pragma once

#include <libipc/ipc.h>
#include <motion/motion_client/robosub_motion_client.h>

#include <memory>
#include <vector>
#include <string>

class Mission
{
public:
    Mission(ipc::Communicator& communicator);
    ~Mission();

    void init_ipc();
    void publish_commands();

    template<typename T>
    void handle_message(const T& msg)
    {
        std::cout << "Message " << ros::message_traits::datatype<T>() << " received" << std::endl;
        std::cout << msg << std::endl;
    }

    static const std::string NODE_NAME;
private:
    std::vector<ros::Publisher> publishers_;

    ipc::Communicator& communicator_;
    RobosubMotionClient motion_;
};