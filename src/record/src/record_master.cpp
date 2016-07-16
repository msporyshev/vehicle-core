#include <string>

#include <ros/ros.h>
#include <libipc/ipc.h>

#include <boost/regex.hpp>
#include <boost/filesystem.hpp>
#include <boost/thread.hpp>

#include <ros/package.h>

#include <time.h>

#include <std_msgs/String.h>
#include <rosbag/recorder.h>
#include <rosbag/player.h>

#include <sstream>
#include <iomanip>

#include "record/CmdRecordPath.h"
#include "record/MsgReady.h"

#define MAX_ATTEMPT 10

int bag_size_ = 0;
int bag_increment_ = 0;
ros::Publisher cmd_pub_;

template <typename T>
std::string to_str(const T value, const int w = 2)
{
    std::ostringstream out;
    if(w != 0) {
        out << std::setfill ('0') << std::setw(w);
    }
    out << value;

    return out.str();
}

void process_msg(const record::MsgReady& msg)
{
    ROS_INFO_STREAM("Answer received!");
    bag_increment_++;
}

void init_ipc(ipc::Communicator& comm)
{
    cmd_pub_ = comm.advertise_cmd<record::CmdRecordPath>("record");
    
    comm.subscribe<record::MsgReady>("record", process_msg, 20);
}

void read_config()
{
    XmlRpc::XmlRpcValue bags;

    ROS_ASSERT(ros::param::get(ros::this_node::getName() + "/bags", bags));
    bag_size_ = bags.size();
}

int main(int argc, char** argv)
{
    auto communicator = ipc::init(argc, argv, "record_master");
     
    init_ipc(communicator);
    read_config();

    time_t theTime = time(NULL);
    struct tm *aTime = localtime(&theTime);

    int day = aTime->tm_mday;
    int month = aTime->tm_mon + 1;
    int year = aTime->tm_year + 1900;

    int hour = aTime->tm_hour;
    int min = aTime->tm_min;
    int sec = aTime->tm_sec;

    std::string day_directory = to_str(year, 4) + "-" + to_str(month) + "-" + to_str(day);
    std::string run_directory = "run_" + to_str(hour) + "-" + to_str(min) + "-" + to_str(sec);

    std::string base_path_ = ros::package::getPath("record");
    std::size_t pos = base_path_.find("/src/record");
    base_path_ = base_path_.substr(0, pos) + "/storage/";    
    

    std::string day_path = base_path_ + day_directory;
    std::string run_path = day_path + "/" + run_directory;

    boost::filesystem::path dir_day(day_path);
    ROS_INFO_STREAM("day path:" << day_path);
    if(boost::filesystem::create_directory(dir_day)) {
        ROS_INFO_STREAM("creating success");
    } else {
        ROS_INFO_STREAM("creating unsuccess");
    }
    
    boost::filesystem::path dir_run(run_path);
    ROS_INFO_STREAM("run path:" << run_path);
    if(boost::filesystem::create_directory(dir_run)) {
        ROS_INFO_STREAM("creating success");
    } else {
        ROS_INFO_STREAM("creating unsuccess");
    }

    int inc = 0;
    ipc::EventLoop loop(1);

    record::CmdRecordPath msg;
    msg.folder = run_path;

    while(loop.ok()){
        if(bag_increment_ == bag_size_) {
            ROS_INFO_STREAM("ALL record nodes successful started");
            break;
        } else {
            inc++;
            ROS_INFO_STREAM("publish cmd");
            cmd_pub_.publish(msg);
            if(inc >= MAX_ATTEMPT) {
                ROS_WARN("Only %d nodes of %d was started!", bag_increment_, bag_size_);
                break;
            }
        }
    }

    return 0;
}