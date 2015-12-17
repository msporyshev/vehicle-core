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

using namespace std;

#define PUBLISH_PERIOD 0.1

int main(int argc, char* argv[])
{
    auto communicator = ipc::init(argc, argv, Dvl::NODE_NAME);

    Dvl dvl;

    dvl.init_connection(communicator);

    communicator.create_timer(PUBLISH_PERIOD, &Dvl::publish_height, &dvl);
    communicator.create_timer(PUBLISH_PERIOD, &Dvl::publish_distance, &dvl);
    communicator.create_timer(PUBLISH_PERIOD, &Dvl::publish_velocity, &dvl);

    ros::spin();
    return 0;
}

///@}