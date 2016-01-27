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

vector<string> split(const string& source, char delimeter)
{
    stringstream ss(source);
    string str;
    vector<string> result;
    while(getline(ss, str, delimeter)) {
        result.push_back(str);
    }

    return result;
}

int main(int argc, char *argv[])
{
    string bin_dir(argv[0]);
    vector<string> dirs(split(bin_dir, '/'));
    bin_dir.clear();
    for (size_t i = 0; i < dirs.size() - 1; ++i) {
        bin_dir += dirs[i] + "/";
    }
    bin_dir += "config/";

    auto communicator = ipc::init(argc, argv, MotionServer::NODE_NAME);

    for (int i = 1; i < NSIG; i++) {
        signal(i, sig_handler);
    }

    YamlReader motion_config("motion.yml", bin_dir);

    MotionServer server(communicator, motion_config);
    server.init_ipc();
    server.run();

    // Central::deinit();

    return 0;
}
