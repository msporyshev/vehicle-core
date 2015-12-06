#include "ros/ros.h"
#include <libipc/ipc.h>

#include <boost/program_options.hpp>
#include <yaml_reader.h>

#include "sidesonar/sidesonar.h"

using namespace std;

#define PUBLISH_PERIOD 0.1

int main(int argc, char* argv[])
{
    auto communicator = ipc::init(argc, argv, Sidesonar::NODE_NAME);

    Sidesonar sidesonar;

    sidesonar.init_connection(communicator);

    communicator.create_timer(PUBLISH_PERIOD, &Sidesonar::publish_line, &sidesonar);

    ros::spin();
    return 0;
}