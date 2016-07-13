#include "ipc.h"

using namespace std;

namespace ipc {

const int Communicator::MSG_QUEUE_SIZE = 1;
const int Communicator::CMD_QUEUE_SIZE = 5;

Communicator init(int argc, char** argv, string node_name) {
    ros::init(argc, argv, node_name);
    return Communicator(node_name);
}

} // namespace ipc
