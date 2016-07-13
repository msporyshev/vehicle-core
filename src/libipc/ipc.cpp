#include "ipc.h"
#include <signal.h>


using namespace std;

namespace ipc {

const int Communicator::MSG_QUEUE_SIZE = 1;
const int Communicator::CMD_QUEUE_SIZE = 5;

Communicator init(int argc, char** argv, string node_name) {
    for (int i = 1; i < NSIG; i++) {
        signal(i, sig_handler);
    }

    ros::init(argc, argv, node_name);
    return Communicator(node_name);
}

} // namespace ipc
