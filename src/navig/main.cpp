#include <utils/node_utils.h>
#include <utils/utils.h>
#include <libipc/ipc.h>
#include <iostream>

#include <supervisor/MsgDepth.h>
#include <dvl/MsgDown.h>
#include <dvl/MsgPlaneVelocity.h>
#include <compass/MsgAngle.h>
#include <compass/MsgAngleRate.h>
#include <compass/MsgAcceleration.h>

#include <navig/MsgDepth.h>
#include <navig/MsgHeight.h>
#include <navig/MsgPlaneVelocity.h>
#include <navig/MsgAngle.h>
#include <navig/MsgAngleRate.h>
#include <navig/MsgLocalPosition.h>
#include <navig/MsgRaw.h>

#include "navig.h"

using namespace this_node;
using namespace this_node::ipc;

int main(int argc, char* argv[])
{
    this_node::init(argc, argv);

    const double RATE = config::read_as<double>("rate");
    const double SLEEP = config::read_as<double>("sleep");
    const double AGE_MAX = config::read_as<double>("age_max");
    const double ATTENUATION = config::read_as<double>("attenuation");

    auto sub_supervisor_depth = subscribe<supervisor::MsgDepth>("supervisor");
    auto sub_dvl_down = subscribe<dvl::MsgDown>("dvl");
    auto sub_dvl_plane_velocity = subscribe<dvl::MsgPlaneVelocity>("dvl");
    auto sub_compass_angle = subscribe<compass::MsgAngle>("compаss");
    auto sub_compass_angle_rate = subscribe<compass::MsgAngleRate>("compаss");
    auto sub_compass_acceleration = subscribe<compass::MsgAcceleration>("compаss");

    auto pub_navig_depth = advertise<navig::MsgDepth>();
    auto pub_navig_height = advertise<navig::MsgHeight>();
    auto pub_navig_plane_velocity = advertise<navig::MsgPlaneVelocity>();
    auto pub_navig_angle = advertise<navig::MsgAngle>();
    auto pub_navig_angle_rate = advertise<navig::MsgAngleRate>();
    auto pub_navig_local_position = advertise<navig::MsgLocalPosition>();
    auto pub_navig_raw = advertise<navig::MsgRaw>();

    navig::MsgDepth navig_depth;
    navig::MsgHeight navig_height;
    navig::MsgPlaneVelocity navig_plane_velocity;
    navig::MsgAngle navig_angle;
    navig::MsgAngleRate navig_angle_rate;
    navig::MsgLocalPosition navig_local_position;
    navig::MsgRaw raw;

    ::ipc::EventLoop loop(RATE);
    while(loop.ok()) {
        double time_now = timestamp();
        static double time_was = time_now;
        double dt = time_now - time_was;
        time_was = time_now;

        auto supervisor_depth     = sub_supervisor_depth    .msg();
        auto dvl_down             = sub_dvl_down            .msg();
        auto dvl_plane_velocity   = sub_dvl_plane_velocity  .msg();
        auto compass_angle        = sub_compass_angle       .msg();
        auto compass_angle_rate   = sub_compass_angle_rate  .msg();
        auto compass_acceleration = sub_compass_acceleration.msg();

        update_age_info(raw.supervisor_depth    , sub_supervisor_depth    .age(), AGE_MAX);
        update_age_info(raw.dvl_down            , sub_dvl_down            .age(), AGE_MAX);
        update_age_info(raw.dvl_plane_velocity  , sub_dvl_plane_velocity  .age(), AGE_MAX);
        update_age_info(raw.compass_angle       , sub_compass_angle       .age(), AGE_MAX);
        update_age_info(raw.compass_angle_rate  , sub_compass_angle_rate  .age(), AGE_MAX);
        update_age_info(raw.compass_acceleration, sub_compass_acceleration.age(), AGE_MAX);

        if(raw.compass_acceleration.fresh) {
            integrate_plane(raw.velocity_acc,
                compass_acceleration, dt);
        }

        raw.velocity_flag = try_get_velocity(
            raw.velocity,
            raw.compass_acceleration.fresh,
            raw.velocity_acc,
            raw.dvl_plane_velocity.fresh,
            dvl_plane_velocity
        );

        integrate_local(raw.position_dvl, dvl_plane_velocity, compass_angle.heading, dt);
        integrate_local(raw.position_acc, raw.velocity_acc  , compass_angle.heading, dt);

        raw.position_flag = (raw.velocity_flag && raw.compass_angle.fresh);
        if(raw.position_flag) {
            integrate_local(raw.position, raw.velocity, compass_angle.heading, dt);
        }


        navig_depth.depth = supervisor_depth.depth;
        navig_depth.velocity = supervisor_depth.velocity;
        navig_height.height = dvl_down.height;
        navig_height.velocity = dvl_down.velocity;
        navig_angle.heading = compass_angle.heading ;
        navig_angle.pitch = compass_angle.pitch;
        navig_angle.roll = compass_angle.roll;
        navig_angle_rate.rate_heading = compass_angle_rate.rate_head;
        navig_angle_rate.rate_pitch = compass_angle_rate.rate_pitch;
        navig_angle_rate.rate_roll = compass_angle_rate.rate_roll;
        navig_plane_velocity.right = raw.velocity.right;
        navig_plane_velocity.forward = raw.velocity.forward;
        navig_local_position.east = raw.position.east;
        navig_local_position.north = raw.position.north;


        pub_navig_raw.publish(raw);
        if(raw.supervisor_depth.fresh) pub_navig_depth.publish(navig_depth);
        if(raw.dvl_down.fresh) pub_navig_height.publish(navig_height);
        if(raw.compass_angle.fresh) pub_navig_angle.publish(navig_angle);
        if(raw.compass_angle_rate.fresh) pub_navig_angle_rate.publish(navig_angle_rate);
        if(raw.velocity_flag) pub_navig_plane_velocity.publish(navig_plane_velocity);
        if(raw.position_flag) pub_navig_local_position.publish(navig_local_position);
    }

    return 0;
}