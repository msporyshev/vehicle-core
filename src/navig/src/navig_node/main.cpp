#include "navig.h"

#include <dynamic_reconfigure/server.h>

int main(int argc, char* argv[])
{
    auto communicator = ipc::init(argc, argv, navig::NODE_NAME);
    navig::init_ipc(communicator);

    auto imu_angle = communicator.subscribe<compass::MsgCompassAngle>("compass");
    auto imu_acc = communicator.subscribe<compass::MsgCompassAcceleration>("compass");
    auto imu_rate = communicator.subscribe<compass::MsgCompassAngleRate>("compass");
    auto est_position = communicator.subscribe<navig::MsgEstimatedPosition>("local_position_estimator");
    auto dvl_dist = communicator.subscribe<dvl::MsgDvlDistance>("dvl");
    auto dvl_vel = communicator.subscribe<dvl::MsgDvlVelocity>("dvl");
    auto dvl_height = communicator.subscribe<dvl::MsgDvlHeight>("dvl");
    auto gps_coord = communicator.subscribe<gps::MsgGpsCoordinate>("gps");
    auto gps_sat = communicator.subscribe<gps::MsgGpsSatellites>("gps");
    auto gps_utc = communicator.subscribe<gps::MsgGpsUtc>("gps");
    auto supervisor_depth = communicator.subscribe<supervisor::MsgSupervisorDepth>("supervisor");

    dynamic_reconfigure::Server<navig::NavigConfig> server;
    dynamic_reconfigure::Server<navig::NavigConfig>::CallbackType node_callback;

    node_callback = boost::bind(&navig::read_config, _1, _2);
    server.setCallback(node_callback);

    ipc::EventLoop loop(navig::get_period());
    while(loop.ok()) {
        if (imu_angle.ready()) {
            navig::handle_angles(*imu_angle.msg());
        }

        if (imu_acc.ready()) {
            navig::handle_acceleration(*imu_acc.msg());
        }

        if (imu_rate.ready()) {
            navig::handle_rate(*imu_rate.msg());
        }

        if (supervisor_depth.ready()) {
            navig::handle_depth(*supervisor_depth.msg());
        }

        if (est_position.ready()) {
            navig::handle_position(*est_position.msg());
        }

        if (dvl_dist.ready()) {
            navig::handle_message(*dvl_dist.msg());
        }

        if (dvl_vel.ready()) {
            navig::handle_velocity(*dvl_vel.msg());
        }

        if (dvl_height.ready()) {
            navig::handle_message(*dvl_height.msg());
        }

        if (gps_coord.ready()) {
            navig::handle_message(*gps_coord.msg());
        }

        if (gps_sat.ready()) {
            navig::handle_message(*gps_sat.msg());
        }

        if (gps_utc.ready()) {
            navig::handle_message(*gps_utc.msg());
        }

        // navig::create_and_publish_depth();
        // navig::create_and_publish_height();
        // navig::create_and_publish_velocity();
    }

    return 0;
}