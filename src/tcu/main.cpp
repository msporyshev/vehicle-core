#include "tcu_surface.h"

int main(int argc, char **argv)
{
	auto communicator = ipc::init(argc, argv, Tcu::NODE_NAME);
    TcuSurface tcu(communicator);    

    ipc::EventLoop loop(20);
    while(loop.ok())
    {
        tcu.increase_loop_params();

        if (tcu.need_stop_thrusters()) {
            ROS_INFO_STREAM("send_thrusts signal");
            tcu.stop_thrusters();
            tcu.send_thrusts();
            tcu.reset_regul_msg_it();
        }

        tcu.routine();        
    }

    return 0;

}