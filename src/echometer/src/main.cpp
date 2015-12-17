/**
\file
\brief main файл Эхолота

В данном файле находятся main-функция нода Эхолота

\defgroup echometer_node Эхолот
\brief Данный нод предназначен для инициализации соединения с Эхолотом, 
корректного выключения, а так же получения данных от эхолота и их публикации в сеть.
*/

///@{

#include "ros/ros.h"
#include <libipc/ipc.h>

#include <boost/program_options.hpp>
#include <yaml_reader.h>

#include "echometer/echometer.h"

using namespace std;

#define PUBLISH_PERIOD 0.1

int main(int argc, char* argv[])
{
    auto communicator = ipc::init(argc, argv, Echometer::NODE_NAME);

    Echometer echometer;

    echometer.init_connection(communicator);

    communicator.create_timer(PUBLISH_PERIOD, &Echometer::publish_height, &echometer);
    communicator.create_timer(PUBLISH_PERIOD, &Echometer::publish_temperature, &echometer);

    ros::spin();
    return 0;
}

///@}