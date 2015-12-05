#include "ros/ros.h"
#include <libipc/ipc.h>

#include <boost/program_options.hpp>
#include <yaml_reader.h>

#include "supervisor.h"

#define PUBLISH_PERIOD 0.1

using namespace std;

int main(int argc, char* argv[])
{
    auto communicator = ipc::init(argc, argv, Supervisor::NODE_NAME);

    Supervisor supervisor;

    supervisor.init_connection(communicator);

    communicator.create_timer(PUBLISH_PERIOD, &Supervisor::publish_leak, &supervisor);
    communicator.create_timer(PUBLISH_PERIOD, &Supervisor::publish_compensator, &supervisor);
    communicator.create_timer(PUBLISH_PERIOD, &Supervisor::publish_devices_status, &supervisor);
    communicator.create_timer(PUBLISH_PERIOD, &Supervisor::publish_short_circuit, &supervisor);
    communicator.create_timer(PUBLISH_PERIOD, &Supervisor::publish_adc, &supervisor);
    communicator.create_timer(PUBLISH_PERIOD, &Supervisor::publish_external_adc, &supervisor);
    communicator.create_timer(PUBLISH_PERIOD, &Supervisor::publish_depth, &supervisor);

    ros::spin();
    return 0;
}