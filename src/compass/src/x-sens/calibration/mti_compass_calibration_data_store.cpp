//==============================================================================
//
// Name        : mti_compass_calibration_data_store.c
// Author      : Fedor Dubrovin
// Description : MTi compass (Xsens Technologies B.V.) calibration data store
// Version     : 001
// Modified    : 2012-08-25 13-00
//
//==============================================================================

#include "compass/x-sens/MTI.h"

#include <libipc/ipc.h>
#include "ros/ros.h"
#include <ros/package.h>
#include <boost/program_options.hpp>

#include <fcntl.h>              // File Control Operations

// Плохой файловый дескриптор
#define INVALID_FILE_DESCRIPTOR (-1)

std::string port;
std::string file_name;
std::string file_path;
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
          "Set filename for stored data, default: mfmResults.bin. 
          Base file_path: src/compass/calibration_data.");

    std::string base_path = ros::package::getPath("compass") + "/calibration_data/";
    if(file_name.size() == 0) {
        file_name = "mfmResults.bin";
    }
    file_path = base_path + file_name;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    if (vm.count("help")) {
        std::cout << desc << std::endl;
        exit(EXIT_SUCCESS);
    }
}

int main ( int argc, char *argv[] )
{
    program_options_init(argc, argv);

    if (port.size() == 0 || baundrate == 0) {
        ROS_ERROR_STREAM("The settings have not been established. Program close.");
        return (EXIT_FAILURE);
    }

    MTI mti;
    
    // Буфер, в который считываются принимаемые с COM-порта данные
    unsigned char buffer[8192];

    com_descriptor = mti.MTI_COM_open(port.c_str(), baundrate);

    // Файловый дескриптор для файла данных
    int fd_calibration = INVALID_FILE_DESCRIPTOR;

  // Открытие файла с данными для калибровки компаса
    fd_calibration = open (file_path.c_str(), O_RDONLY);

    if ( fd_calibration == INVALID_FILE_DESCRIPTOR ) {
        ROS_ERROR_STREAM("ERROR: Calibration file " << file_name << " is absent");
        ROS_ERROR_STREAM("Program was terminated");

        // Закрытие COM-порта
        mti.MTI_COM_close (&com_descriptor);
        return (EXIT_FAILURE);
    }

    ROS_INFO_STREAM("Calibration file mfmResults.bin was opened");

    int n = read (fd_calibration, buffer, 65);

    if (n != 65) {
        ROS_ERROR_STREAM("ERROR: Calibration file " << file_name << " reading, size: " << n);

        close (fd_calibration);
        return (EXIT_FAILURE);
    }
    ROS_INFO_STREAM("Calibration file reading: OK");

    n = write (com_descriptor, mti.GoToConfig, mti.GoToConfig_size);
    usleep (500000);

    n = write (com_descriptor, buffer, 65);
    usleep (500000);

    if (n != 65) {
        ROS_ERROR_STREAM("ERROR: Calibration data writing error");

        // Закрытие COM-порта
        mti.MTI_COM_close(&com_descriptor);
        return (EXIT_FAILURE);
    }

    ROS_INFO_STREAM("Calibration was successfully finished");

    close (fd_calibration);
    ROS_INFO_STREAM("Calibration file was closed");

    // Закрытие COM-порта
    mti.MTI_COM_close (&com_descriptor);

    return (EXIT_SUCCESS);
}