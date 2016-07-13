#include "motion_server.h"

#include <config_reader/yaml_reader.h>
#include <signal.h>
#include <sstream>
#include <vector>
#include <string>

// #include "ipc_lib.h"
#include <libauv/utils/basic.h>

#include <iostream>

using namespace std;

int main(int argc, char *argv[])
{
    auto communicator = ipc::init(argc, argv, MotionServer::NODE_NAME);

    YamlReader motion_config("motion.yml", MotionServer::NODE_NAME);

    MotionServer server(communicator, motion_config);
    server.init_ipc();
    server.run();

    // Central::deinit();

    return 0;
}
