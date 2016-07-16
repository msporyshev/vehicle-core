#include <string>
#include <signal.h>

#include <ros/ros.h>
#include <libipc/ipc.h>

#include <boost/regex.hpp>
#include <boost/filesystem.hpp>
#include <boost/thread.hpp>

#include <ros/package.h>

#include <std_msgs/String.h>
#include <rosbag/recorder.h>
#include <rosbag/player.h>

#include "record/CmdRecordPath.h"
#include "record/MsgReady.h"

struct bag
{
    bool node_base;
    std::string name;
    std::vector<std::string> include_topics;
    std::vector<std::string> exclude_topics;
};

ros::Publisher msg_pub_;
int default_block_size_;
std::string default_archive_mode_;
std::string bagname_;
std::string bag_folder_;
bool cmd_received = false;
bag bagfile;

void init_ipc(ipc::Communicator& comm);
void read_config();
void set_record(bag bg);

void process_cmd(const record::CmdRecordPath& cmd);

void init_ipc(ipc::Communicator& comm)
{
    msg_pub_ = comm.advertise<record::MsgReady>();
    comm.subscribe_cmd<record::CmdRecordPath>(process_cmd);
}

void process_cmd(const record::CmdRecordPath& cmd) 
{
    if(cmd_received) {
        return;
    }

    ROS_INFO_STREAM("receive folder: " << cmd.folder);
    bag_folder_ = cmd.folder;
    cmd_received = true;

    record::MsgReady msg;
    msg.ready = true;
    msg_pub_.publish(msg);
}

std::string get_union_regex(std::vector<std::string> topics) 
{
    std::string out = "";   
    for(auto &topic: topics) {
        out += "(" + topic + ")";
        if(topic != *(topics.end() - 1)) {
            out += "|";
        }
    }
    ROS_DEBUG_STREAM("regex: " << out);
    return out;
}

void customSigIntHandler(int sig)
{
  ros::shutdown();
}

void read_config()
{
    XmlRpc::XmlRpcValue bags;

    ROS_ASSERT(ros::param::get(ros::this_node::getName() + "/bagname", bagname_));

    // ROS_ASSERT(ros::param::get("/record/settings/default_block_size", default_block_size_));
    ROS_ASSERT(ros::param::get(ros::this_node::getName() + "/bags", bags));
    ROS_DEBUG_STREAM("name:" << bagname_);

    int bag_index = -1;

    for(int i = 0; i < bags.size(); i++) {
        
        ROS_ASSERT_MSG(bags[i]["name"].valid(), "Bag name not exist at %d index", i);
        bagfile.name = static_cast<std::string>(bags[i]["name"]);
        
        if(bagfile.name == bagname_) {
            bag_index = i;
            break;
        }
    }
    ROS_ASSERT_MSG(bag_index >= 0, "Bag settings not found");
    ROS_DEBUG_STREAM("bag index: " << bag_index);


    if(bags[bag_index]["params"]["node_base"].valid()) {
        bagfile.node_base    = static_cast<bool>(bags[bag_index]["params"]["node_base"]);
    } else {
        ROS_WARN("node_base not exist at index. Set default.");
        bagfile.node_base = true;
    }

    if(bags[bag_index]["params"]["include_topics"].valid()) {
        XmlRpc::XmlRpcValue include_topics = bags[bag_index]["params"]["include_topics"];
        for(int i = 0; i < include_topics.size(); i++) {
            bagfile.include_topics.push_back(static_cast<std::string>(include_topics[i]));
        }
    } else {
        ROS_WARN("include_topics not exist. Set empty.");
    }

    if(bags[bag_index]["params"]["exclude_topics"].valid()) {
        XmlRpc::XmlRpcValue exclude_topics = bags[bag_index]["params"]["exclude_topics"];
        for(int i = 0; i < exclude_topics.size(); i++) {
            bagfile.exclude_topics.push_back(static_cast<std::string>(exclude_topics[i]));
        }
    } else {
        ROS_WARN("include_topics not exist at index. Set empty.");
    }

    ROS_INFO_STREAM("name:" << bagfile.name);
    ROS_INFO_STREAM("is node:" << bagfile.node_base);
    ROS_INFO_STREAM("include_topics:");

    for(auto &topic: bagfile.include_topics) {
        ROS_INFO_STREAM("\t - " << topic);
    }
    ROS_INFO_STREAM("exclude_topics:");
    for(auto &topic: bagfile.exclude_topics) {
        ROS_INFO_STREAM("\t - " << topic);
    }
}

void set_record(bag bg) {
    rosbag::RecorderOptions options;
    options.record_all = false;
    options.append_date = false;
    options.regex = true;
    options.compression = rosbag::compression::BZ2;
    
    if(bg.exclude_topics.size() > 0) {
        options.do_exclude = true;
        options.exclude_regex = boost::regex(get_union_regex(bg.exclude_topics));
    }
    
    if(bg.include_topics.size() > 0) {
        for(auto &topic: bg.include_topics) {
            options.topics.push_back(topic);
        }
    }
    if(bg.node_base) {
        options.topics.push_back("/" + bg.name + "(.*)");
    }

    options.prefix = bag_folder_ + "/" + bg.name;
    ROS_DEBUG_STREAM("path to save: " << options.prefix);
    rosbag::Recorder* record = new rosbag::Recorder(options);
    record->run();
}

int main(int argc, char** argv)
{
    auto communicator = ipc::init(argc, argv, "record");
    signal(SIGINT, customSigIntHandler);
    
    init_ipc(communicator);
    read_config();
    
    ipc::EventLoop loop(10);
    while(loop.ok()){
        if(cmd_received) {
            set_record(bagfile);
        }
    }

    return 0;
}