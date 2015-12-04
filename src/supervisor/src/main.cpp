#include "ros/ros.h"
#include <libipc/ipc.h>

#include <boost/program_options.hpp>
#include <yaml_reader.h>

#include "supervisor.h"

using namespace std;

int main(int argc, char* argv[])
{
    auto communicator = ipc::init(argc, argv, Supervisor::NODE_NAME);

    Supervisor supervisor;

    supervisor.init_connection(communicator);

    cout << "try to start" << endl;

    communicator.create_timer(0.1, &Supervisor::publish_leak, &supervisor);
    communicator.create_timer(0.1, &Supervisor::publish_compensator, &supervisor);
    communicator.create_timer(0.1, &Supervisor::publish_devices_status, &supervisor);
    communicator.create_timer(0.1, &Supervisor::publish_short_circuit, &supervisor);
    communicator.create_timer(0.1, &Supervisor::publish_adc, &supervisor);
    communicator.create_timer(0.1, &Supervisor::publish_external_adc, &supervisor);
    communicator.create_timer(0.1, &Supervisor::publish_depth, &supervisor);

    cout << "try to rotate" << endl;
    ros::spin();
    cout << "try to end" << endl;
    return 0;
}