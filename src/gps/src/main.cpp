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