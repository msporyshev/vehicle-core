#include "compass/x-sens/compass.h"

#include <iostream>

#include <libipc/ipc.h>
#include "ros/ros.h"
#include "std_msgs/String.h"

#include <boost/program_options.hpp>

#define DEFAULT_COM_PORT "/dev/ttyS0"
#define DEFAULT_BAUDRATE 57600

using namespace std;
namespace po = boost::program_options;

void program_options_init(int argc, char** argv, CompassConfig& config) 
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
      ("declination,d", po::value(&(config.declination)),
          "Set magnetic declination in degrees (e.g. -d -4.9).");

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    if (vm.count("help")) {
      std::cout << desc << std::endl;
      exit(EXIT_SUCCESS);
    }
}

void read_config(CompassConfig& config)
{
    ROS_ASSERT(ros::param::get("/compass/connection/com_port", config.port));
    ROS_ASSERT(ros::param::get("/compass/connection/baudrate", config.baudrate));
    ROS_ASSERT(ros::param::get("/compass/compass_config/declination", config.declination));
    ROS_ASSERT(ros::param::get("/compass/compass_config/modelling", config.modelling));
}

int main (int argc, char *argv[])
{

    CompassConfig config;

    config.port = DEFAULT_COM_PORT;
    config.baudrate = DEFAULT_BAUDRATE;

    auto communicator = ipc::init(argc, argv, Compass::NODE_NAME);

    read_config(config);
    program_options_init(argc, argv, config);

    if ((config.port.size() == 0 || config.baudrate == 0) && !config.modelling) {
        ROS_ERROR_STREAM("The settings have not been established. Program close.");
        return 1;
    }

    Compass compass(config);

    compass.init_connection(communicator);
    compass.init_mti();  

    compass.start_timers(communicator);

    ros::spin();

    compass.close_mti();

    ROS_INFO_STREAM("Program was terminated.");
    return (EXIT_SUCCESS);
}