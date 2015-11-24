#include <ros/ros.h>
#include <std_msgs/String.h>

#include <libipc/ipc.h>

using namespace std;

int main(int argc, char** argv)
{
    auto comm = ipc::init(argc, argv, "ipcpub");

    auto job_pub = comm.advertise_cmd<std_msgs::String>("worker");
    auto info_pub = comm.advertise<std_msgs::String>();

    ipc::EventLoop loop(10); // 10 hz
    while (loop.ok()) {
        std_msgs::String info, cmd;
        info.data = "my data";
        cmd.data = "WHERE IS MY COKE?";

        job_pub.publish(cmd);
        info_pub.publish(info);
    }
}