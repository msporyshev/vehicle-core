/**
\file
\brief main файл GPS

В данном файле находятся main-функция нода GPS

\defgroup gps_node GPS
\brief Данный нод предназначен для инициализации соединения с GPS, 
корректного выключения, а так же получения данных от GPS и их публикации в сеть.
*/

///@{
#include "ros/ros.h"
#include <libipc/ipc.h>

#include <boost/program_options.hpp>
#include <yaml_reader.h>

#include "gps/gps.h"

using namespace std;

#define DEFAULT_COM_PORT "/dev/ttyS0"
#define DEFAULT_BAUDRATE 57600
#define PUBLISH_PERIOD 1
#define HANDLE_PERIOD 0.02

using namespace std;
namespace po = boost::program_options;



void program_options_init(int argc, char** argv, GpsConfig& config) 
{
    po::options_description desc("Usage");
    desc.add_options()
      ("help,h", "Produce help message.")
      ("port,c", po::value(&(config.port)),
          "Set COM-port name (e.g. /dev/ttyUSB0).")
      ("baundrate,b", po::value(&(config.baudrate)),
          "Set COM-port baundrate (e.g. -b 115200).")
      ("simulating,o", po::value(&(config.simulating))->zero_tokens(),
          "Set simulating mode.");

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    if (vm.count("help")) {
      std::cout << desc << std::endl;
      exit(EXIT_SUCCESS);
    }
}

void read_config(GpsConfig& config)
{
    ros::param::get("/gps/connection/com_port", config.port);
    ros::param::get("/gps/connection/baudrate", config.baudrate);
    ros::param::get("/gps/config/simulating", config.simulating);
}

int main(int argc, char* argv[])
{
    GpsConfig config;

    config.port = DEFAULT_COM_PORT;
    config.baudrate = DEFAULT_BAUDRATE;

    auto communicator = ipc::init(argc, argv, Gps::NODE_NAME);
    
    read_config(config);
    program_options_init(argc, argv, config);
    
    if ((config.port.size() == 0 || config.baudrate == 0) && !config.simulating) {
        ROS_ERROR_STREAM("The settings have not been established. Program close.");
        return 1;
    }

    Gps gps(config);

    gps.init_connection(communicator);

    if(!config.simulating) {
        communicator.create_timer(HANDLE_PERIOD,  &Gps::handle_gps_data, &gps);
    } else {
        communicator.create_timer(PUBLISH_PERIOD, &Gps::publish_sim_global_position, &gps);
        communicator.create_timer(PUBLISH_PERIOD, &Gps::publish_sim_satellites, &gps);
        communicator.create_timer(PUBLISH_PERIOD, &Gps::publish_sim_utc, &gps);
        communicator.create_timer(PUBLISH_PERIOD, &Gps::publish_sim_raw, &gps);
    }

    if(!config.simulating) {
        gps.open_device();
    }

    ros::spin();
    
    if(!config.simulating) {
        gps.close_device();
    }   
    
    ROS_INFO_STREAM("Program was terminated."); 
    return 0;
}

///@}