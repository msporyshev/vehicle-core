//==============================================================================
//
// Name        : mti_compass_calibration_data_collect.c
// Author      : Fedor Dubrovin
// Description : MTi compass (Xsens Technologies B.V.) calibration data collect
// Version     : 001
// Modified    : 2012-08-25 12-00
//
//==============================================================================

// Период таймера в микросекундах
#define TIMER_PERIOD (10000)

// Плохой файловый дескриптор
#define INVALID_FILE_DESCRIPTOR (-1)

//------------------------------------------------------------------------------

#include "compass/x-sens/MTI.h"

#include <libipc/ipc.h>
#include "ros/ros.h"
#include <ros/package.h>
#include <boost/program_options.hpp>

#include <fcntl.h>              // File Control Operations

std::string port;
std::string file_name;
std::string file_path;
int calibration_time;
int baundrate;

int com_descriptor;

using namespace std;
namespace po = boost::program_options;

void program_options_init(int argc, char** argv) 
{
    po::options_description desc("Usage");
    desc.add_options()
      ("help,h", "Produce help message.")
      ("port,c", po::value(&port),
          "Set COM-port name (e.g. /dev/ttyS0).")
      ("baundrate,b", po::value(&baundrate),
          "Set COM-port baundrate (e.g. -b 115200).")
      ("file,f", po::value(&file_name),
          "Set filename for data.")
      ("time,t", po::value(&calibration_time),
          "Set calibration period, sec. (e.g. -t 100).");

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    if (vm.count("help")) {
        std::cout << desc << std::endl;
        exit(EXIT_SUCCESS);
    }

    std::string base_path = ros::package::getPath("compass") + "/calibration_data/";
    if(file_name.size() == 0) {
        file_name = "MT_data.mtb";
    }
    file_path = base_path + file_name;
}

void read_config()
{
    ros::param::get("/compass/connection/baundrate", baundrate);
    ros::param::get("/compass/connection/com_port", port);
    ros::param::get("/compass/calibrate_collect/file_name", file_name);
    ros::param::get("/compass/calibrate_collect/calibration_period", calibration_time);
}

int main ( int argc, char *argv[] )
{
    auto communicator = ipc::init(argc, argv, "compass");
    
    read_config();
    
    program_options_init(argc, argv);

    ROS_ASSERT_MSG(( !(port.size() == 0 || baundrate == 0)), 
        "The settings have not been established. Program close.");

    MTI mti;
    
    // Буфер, в который считываются принимаемые с COM-порта данные
    unsigned char buffer[8192];

    com_descriptor = mti.MTI_COM_open(port.c_str(), baundrate);

    // Файловый дескриптор для файла данных
    int fd_raw = INVALID_FILE_DESCRIPTOR;

    // Создание файла с сырыми данными для калибровки компаса
    fd_raw = open(file_path.c_str(), O_WRONLY | O_CREAT);

    if (fd_raw == INVALID_FILE_DESCRIPTOR) {
        ROS_ERROR_STREAM("ERROR: Can not create output file" <<  file_name);
        ROS_ERROR_STREAM("Program was terminated");

        // Закрытие COM-порта
        mti.MTI_COM_close(&com_descriptor);
        return (EXIT_FAILURE);
    }

    mti.MTI_start_calibrate(com_descriptor);
    ROS_INFO_STREAM("Start calibration");

    // Очистка входного буфера COM-порта от полученных ответов
    int n;
    do {
        n = read (com_descriptor, buffer, 8192);
    } while (n > 0);

    ROS_INFO_STREAM("Buffer flush");


    mti.MTI_reset(com_descriptor);
    ROS_INFO_STREAM("Reset compass");

    ros::Time::init();
    ros::Time start_time = ros::Time::now();
    ros::Duration calibration_period = ros::Duration(calibration_time);
    double time_left_last = 0;

    while (ros::Time::now() - start_time < calibration_period) {
        n = read (com_descriptor, buffer, 8192);

        if (n > 0) {
            write (fd_raw, buffer, n);
        } else {
            usleep (TIMER_PERIOD);
        }

        int time_left = calibration_period.toSec() + start_time.toSec() - ros::Time::now().toSec();
        if(!(time_left % 10) && time_left != time_left_last) {
            ROS_INFO_STREAM("Time left: " << time_left << " sec");   
        }
            time_left_last = time_left;
    }

    mti.MTI_stop_calibrate (com_descriptor);
    ROS_INFO_STREAM("Stop calibration");

      // Дописываем в файл все данные из приемного буфера COM-порта
    while ((n = read (com_descriptor, buffer, 8192)) > 0) {
        write (fd_raw, buffer, n);
    }

    close(fd_raw);
    ROS_INFO_STREAM("File MT_data.mtb was closed");

    mti.MTI_COM_close (&com_descriptor);
    ROS_INFO_STREAM("Compass was closed");
}