/**
\file
\brief main файл Супервизора

В данном файле находятся main-функция нода Супервизора

\defgroup supervisor_node Супервизор
\brief Данный нод предназначен для инициализации соединения с Супервизором, 
корректного выключения, а так же получения данных от супервизора, их публикации в сеть,
а так же для управления работой супервизора и изменением режимов его работы.
*/

///@{

#include "ros/ros.h"
#include <libipc/ipc.h>

#include <boost/program_options.hpp>

#include "supervisor/supervisor.h"

namespace po = boost::program_options;

using namespace std;

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
    
    auto communicator = ipc::init(argc, argv, Supervisor::NODE_NAME);

    Supervisor supervisor(is_simulating);
    supervisor.init_connection(communicator);

    supervisor.start_timers(communicator);

    ros::spin();
    return 0;
}

///@}