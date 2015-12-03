#pragma once

#include <libipc/ipc.h>
#include <motion/motion_client/robosub_motion_client.h>

#include <memory>
#include <vector>
#include <string>

class Mission
{
public:
    Mission(ipc::Communicator* communicator);
    ~Mission();

    void init_ipc();
    void publish_commands();

    static const std::string NODE_NAME;
private:
    Mission(const Mission& rhs);

    std::vector<ros::Publisher> publishers_;

    std::shared_ptr<ipc::Communicator> communicator_;
    std::shared_ptr<RobosubMotionClient> motion_;
};