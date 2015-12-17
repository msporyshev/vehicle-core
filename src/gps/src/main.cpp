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

#include "gps.h"

using namespace std;

#define PUBLISH_PERIOD 0.1

int main(int argc, char* argv[])
{
    auto communicator = ipc::init(argc, argv, Gps::NODE_NAME);

    Gps gps;

    gps.init_connection(communicator);

    communicator.create_timer(PUBLISH_PERIOD, &Gps::publish_coordinate, &gps);
    communicator.create_timer(PUBLISH_PERIOD, &Gps::publish_satellites, &gps);
    communicator.create_timer(PUBLISH_PERIOD, &Gps::publish_utc, &gps);

    ros::spin();
    return 0;
}

///@}