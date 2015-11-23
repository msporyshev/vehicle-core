#include <ros/ros.h>
#include <std_msgs/String.h>

#include "ipc.h"

using namespace std;

int main(int argc, char** argv)
{
    auto comm = ipc::init(argc, argv, "ipctest");

    auto sub = comm.subscribe<std_msgs::String>("chatter");

    ros::Rate rate(10);
    cout << sub.msg_wait() << endl;
}