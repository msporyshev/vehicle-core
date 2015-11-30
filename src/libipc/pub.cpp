#include <libipc/ipc.h>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Time.h>
#include <libipc/MsgTestMsg.h>


using namespace std;

int main(int argc, char** argv)
{
    auto comm = ipc::init(argc, argv, "ipcpub");

    auto job_pub = comm.advertise_cmd<std_msgs::String>("worker");
    auto info_pub = comm.advertise<std_msgs::String>();
    auto info_pub2 = comm.advertise<std_msgs::Time>();

    ipc::EventLoop loop(10); // 10 hz
    while (loop.ok()) {
        std_msgs::String info, cmd;
        info.data = "my data";
        cmd.data = "WHERE IS MY COKE?";

        std_msgs::Time info2;


        job_pub.publish(cmd);
        info_pub.publish(info);
        info_pub2.publish(info2);
    }
}