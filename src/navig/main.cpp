#include <utils/node_utils.h>
#include <utils/utils.h>
#include <libipc/ipc.h>

#include <compass/MsgAngle.h>
#include <dvl/MsgPlaneVelocity.h>
#include <dvl/MsgDownwardVelocity.h>
#include <navig/MsgPosition.h>

#include <iostream>
#include <cmath>

int main(int argc, char* argv[])
{
    this_node::init(argc, argv); // Название узла выведется из названия исполняемого файла,
                                 // также выведется и название конфига

    double rate = this_node::config::read_as<double>("rate");
    double angles_age = this_node::config::read_as<double>("angles_age");
    double velocity_age = this_node::config::read_as<double>("velocity_age");

    auto angles_sub = this_node::ipc::subscribe<compass::MsgAngle>("compass");
    auto plane_velocity_sub = this_node::ipc::subscribe<dvl::MsgPlaneVelocity>("dvl");
    auto down_velocity_sub = this_node::ipc::subscribe<dvl::MsgDownwardVelocity>("dvl");
    auto position_pub = this_node::ipc::advertise<navig::MsgPosition>();

    ipc::EventLoop loop(rate);
    while(loop.ok())
    {

        double now = timestamp();
        static double was = now;
        double dt = now - was;

        if(angles_sub.is_actual(angles_age) &&
           plane_velocity_sub.is_actual(velocity_age))
        {
            auto angles = angles_sub.msg();
            auto velocity = plane_velocity_sub.msg();

            navig::MsgPosition pos_msg;
            pos_msg.east  += dt * ( velocity.rightward   * cos(utils::to_rad(angles.heading))
                                      +velocity.forward * sin(utils::to_rad(angles.heading)));
            pos_msg.north += dt * (-velocity.rightward   * sin(utils::to_rad(angles.heading))
                                      +velocity.forward * cos(utils::to_rad(angles.heading)));

            position_pub.publish(pos_msg);
        }
    }

    return 0;
}
