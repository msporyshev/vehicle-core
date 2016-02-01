#include "navig.h"
#include "navig_simulating.h"

#include <dynamic_reconfigure/server.h>

#include <boost/program_options.hpp>

#include <iostream>

namespace po = boost::program_options;

bool program_options_init(int argc, char* argv[])
{
    po::options_description desc("Usage");
    desc.add_options()
        ("help,h", "Produce help message.")
        ("simulating,s", po::bool_switch()->default_value(false), "Enable simulating mode");

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    if (vm.count("help")) {
        std::cout << desc << std::endl;
        exit(EXIT_SUCCESS);
    }

    return vm["simulating"].as<bool>();
}

int main(int argc, char* argv[])
{
    int is_simulating = program_options_init(argc, argv);

    auto communicator = ipc::init(argc, argv, NavigBase::NODE_NAME);

    std::shared_ptr<NavigBase> navig;
    if (is_simulating) {
        navig = std::make_shared<NavigSimulating>();
    } else {
        navig = std::make_shared<Navig>();
    }

    navig->init_ipc(communicator);

    dynamic_reconfigure::Server<navig::NavigConfig> server;
    dynamic_reconfigure::Server<navig::NavigConfig>::CallbackType node_callback;

    node_callback = boost::bind(&NavigBase::read_config, navig.get(), _1, _2);
    server.setCallback(node_callback);

    navig->run();

    return 0;
}
