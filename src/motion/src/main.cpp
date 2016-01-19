#include "motion_server.h"

#include <config_reader/yaml_reader.h>
#include <signal.h>

// #include "ipc_lib.h"
#include <libauv/utils/basic.h>

int main(int argc, char *argv[])
{
    auto communicator = ipc::init(argc, argv, MotionServer::NODE_NAME);

    for (int i = 1; i < NSIG; i++) {
        signal(i, sig_handler);
    }

    YamlReader motion_config("motion.yml");
    MotionServer server(communicator, motion_config);
    server.init_ipc();
    server.run();

    // Central::deinit();

    return 0;
}
