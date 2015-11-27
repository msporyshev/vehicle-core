#include <ros/ros.h>
#include <std_msgs/String.h>

#include <libipc/ipc.h>

#include <iostream>

using namespace std;

void on_job_receive(const std_msgs::String& msg) {
    cout << "my cmd: " << msg << endl;
}

class InfoReader
{
public:
    InfoReader(ipc::Communicator& comm)
    {
        comm.subscribe("ipcpub", &InfoReader::on_info_receive, this);
    }

private:
    void on_info_receive(const std_msgs::String& info)
    {
        cout << "new info: " << info << endl;
    }
};

int main(int argc, char** argv)
{
    auto comm = ipc::init(argc, argv, "worker");

    InfoReader ir(comm);
    comm.subscribe_cmd(on_job_receive);

    ros::spin();
}