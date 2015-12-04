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

    static const std::string NODE_NAME;
private:
    std::vector<ros::Publisher> publishers_;

    ipc::Communicator& communicator_;
    RobosubMotionClient motion_;
};