/**
\file
\brief main-модуль нода Доплера

В данном файле находится main-функция нода доплера

\defgroup dvl_node Доплер
\brief Данный нод предназначен для инициализации соединения с доплеровским лагом, 
корректного выключения, а так же получения данных от доплеровского лага и их публикации в сеть.
*/

///@{

#include "ros/ros.h"
#include <libipc/ipc.h>

#include <boost/program_options.hpp>
#include <yaml_reader.h>

#include "dvl/dvl.h"

#define DEFAULT_PORT        "/dev/ttyS3"
#define DEFAULT_BAUDRATE    115200

using namespace std;
namespace po = boost::program_options;

#define PUBLISH_PERIOD 0.1

void program_options_init(int argc, char** argv, dvlConfig& config) 
{
    po::options_description desc("Usage");
    desc.add_options()
      ("help,h", "Produce help message.")
      ("port,c", po::value(&(config.port)),
          "Set COM-port name (e.g. /dev/ttyUSB0).")
      ("baundrate,b", po::value(&(config.baudrate)),
          "Set COM-port baundrate (e.g. -b 115200).")
      ("modelling,o", po::value(&(config.modelling))->zero_tokens(),
          "Set modelling mode.")
      ("start_now,s", po::value(&(config.start_now))->zero_tokens(),
          "Start dvl without waiting command.");

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    if (vm.count("help")) {
      std::cout << desc << std::endl;
      exit(EXIT_SUCCESS);
    }
}

int main(int argc, char* argv[])
{
    auto communicator = ipc::init(argc, argv, Dvl::NODE_NAME);

    dvlConfig config;

    config.port = DEFAULT_PORT;
    config.baudrate = DEFAULT_BAUDRATE;

    program_options_init(argc, argv, config);

    if ((config.port.size() == 0 || config.baudrate == 0) && !config.modelling) {
        ROS_ERROR_STREAM("The settings have not been established. Program close.");
        return 1;
    }

    Dvl dvl(config);

    dvl.init_connection(communicator);

    dvl.init_dvl();  

    dvl.start_timers(communicator);

    ros::spin();

    dvl.deinit_dvl();

    return 0;
}

///@}