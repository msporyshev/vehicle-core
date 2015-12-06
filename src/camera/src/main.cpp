#include "ros/ros.h"
#include <libipc/ipc.h>

#include <boost/program_options.hpp>
#include <yaml_reader.h>

#include "camera/camera.h"

using namespace std;

#define PUBLISH_PERIOD 0.1

int main(int argc, char* argv[])
{
    auto communicator = ipc::init(argc, argv, Camera::NODE_NAME);

    Camera camera;

    camera.init_connection(communicator);

    communicator.create_timer(PUBLISH_PERIOD, &Camera::publish_frame, &camera);

    ros::spin();
    return 0;
}