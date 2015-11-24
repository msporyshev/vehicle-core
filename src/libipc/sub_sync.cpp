#include <ros/ros.h>
#include <std_msgs/String.h>

#include <libipc/ipc.h>
#include <functional>

#include <iostream>

using namespace std;

void on_job_receive(const std_msgs::String& msg) {
    cout << "job received: " << msg << endl;
}

int main(int argc, char** argv)
{
    auto comm = ipc::init(argc, argv, "worker");

    comm.subscribe_cmd(on_job_receive); // We can use sync and async communication together

    auto cmd = comm.subscribe_cmd<std_msgs::String>();
    auto info = comm.subscribe<std_msgs::String>("ipcpub");


    ipc::EventLoop loop(10); // 10hz

    while (loop.ok()) {
        cout << cmd.msg_wait() << endl;   // Waits only first message

        if (cmd.ready()) {
            cout << *cmd.msg() << endl; // another way
        }

        cout << info.msg_wait() << endl;
    }
}