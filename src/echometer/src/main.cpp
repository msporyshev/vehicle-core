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

#define UPDATE_PERIOD 0.05
#define PUBLISH_PERIOD 0.1

int main(int argc, char* argv[])
{
    auto communicator = ipc::init(argc, argv, Echometer::NODE_NAME);

    Echometer echometer;

    echometer.init_connection(communicator);

    echometer.read_config();

    bool simulate_status, instant_start_status;

    simulate_status = echometer.get_simulate_status();
    instant_start_status = echometer.get_instant_start_status();

    if(!simulate_status && instant_start_status) {
        echometer.connect_device();
    }

    if(!simulate_status) {
        communicator.create_timer(UPDATE_PERIOD,  &Echometer::update_data, &echometer);
        communicator.create_timer(PUBLISH_PERIOD, &Echometer::publish_height, &echometer);
        communicator.create_timer(PUBLISH_PERIOD, &Echometer::publish_temperature, &echometer);
    } else {
        communicator.create_timer(PUBLISH_PERIOD, &Echometer::publish_simulate_height, &echometer);
        communicator.create_timer(PUBLISH_PERIOD, &Echometer::publish_simulate_temperature, &echometer);
    }

    ros::spin();
    return 0;
}

///@}