#include "motion_server.h"

#include <config_reader/yaml_reader.h>
#include <signal.h>
#include <sstream>
#include <vector>
#include <string>

// #include "ipc_lib.h"
#include <libauv/utils/basic.h>
#include <utils/node_utils.h>

#include <iostream>

using namespace std;

int main(int argc, char *argv[])
{
    this_node::init(argc, argv);
    auto& communicator = this_node::comm();

    auto& motion_config = this_node::cfg();

    MotionServer server(communicator, motion_config);
    server.init_ipc();
    server.run();

    return 0;
}
