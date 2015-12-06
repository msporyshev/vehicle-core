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