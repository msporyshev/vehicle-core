#pragma once

#include <string>

#include <libipc/ipc.h>
#include <config_reader/yaml_reader.h>
#include <factory/static_registrator.h>

#include <ros/ros.h>

#include <boost/filesystem.hpp>

namespace this_node {

class Node {
public:
    void init(int argc, char** argv) {
        name = boost::filesystem::path(argv[0]).filename().string();
        comm = new ipc::Communicator(ipc::init(argc, argv, name));
        try {
            cfg = new YamlReader(name + ".yml", name);
        } catch (const YAML::BadFile& e) {
            ROS_INFO_STREAM("" << e.what());
        }
    }

    ipc::Communicator* comm;
    YamlReader* cfg;
    std::string name;

    ~Node() {
        delete comm;
        delete cfg;
    }
};

using ThisNode = Singleton<Node>;

void init(int argc, char** argv) {
    ThisNode::instance().init(argc, argv);
}

Node& node() {
    return ThisNode::instance();
}

const std::string& name() {
    return ThisNode::instance().name;
}

YamlReader& cfg() {
    return *ThisNode::instance().cfg;
}

::ipc::Communicator& comm() {
    return *ThisNode::instance().comm;
}


namespace config {

template<typename T>
T read_as(std::string param_name)
{
    return cfg().read_as<T>(param_name);
}

template<typename T>
bool is_param_readable(T& var, std::string param_name)
{
    return cfg().is_param_readable(var, param_name);
}

template<typename T>
T read_as_throw(std::string name)
{
    return cfg().read_as_throw<T>(name);
}

template<typename T>
T read_param(T& var, std::string param_name)
{
    return cfg().read_param(var, param_name);
}

} // namespace config

namespace ipc {

#define QUEUE_SIZE 5

template<typename Msg>
::ipc::Subscriber<Msg> subscribe(std::string module, int queue_size = QUEUE_SIZE)
{
    return comm().subscribe<Msg>(module, queue_size);
}

template<typename Msg>
::ipc::Subscriber<Msg> subscribe(std::string module, void(*callback)(const Msg&), int queue_size = QUEUE_SIZE)
{
    return comm().subscribe<Msg>(module, callback, queue_size);
}

template<typename Class, typename Msg>
::ipc::Subscriber<Msg> subscribe(std::string module,
        void(Class::*callback)(const Msg&),
        Class* obj,
        int queue_size = QUEUE_SIZE)
{
    return comm().subscribe<Class, Msg>(module, callback, obj, queue_size);
}

template<typename Msg>
::ipc::Subscriber<Msg> subscribe_cmd(int queue_size = QUEUE_SIZE)
{
    return comm().subscribe_cmd<Msg>(queue_size);
}

template<typename Msg>
::ipc::Subscriber<Msg> subscribe_cmd(void(*callback)(const Msg&), int queue_size = QUEUE_SIZE)
{
    return comm().subscribe_cmd<Msg>(callback, queue_size);
}

template<typename Class, typename Msg>
::ipc::Subscriber<Msg> subscribe_cmd(
        void(Class::*callback)(const Msg&),
        Class* obj,
        int queue_size = QUEUE_SIZE)
{
    return comm().subscribe_cmd<Class, Msg>(callback, obj, queue_size);
}

template<typename Msg>
ros::Publisher advertise_cmd(std::string module, int queue_size = QUEUE_SIZE)
{
    return comm().advertise_cmd<Msg>(module);
}

template<typename Msg>
ros::Publisher advertise(int queue_size = QUEUE_SIZE)
{
    return comm().advertise<Msg>(queue_size);
}

template<typename Msg>
ros::Publisher advertise(std::string topic, int queue_size = QUEUE_SIZE)
{
    return comm().advertise<Msg>(topic, queue_size);
}

ros::Timer create_timer(double duration, void(*callback)(const ros::TimerEvent&))
{
    return comm().create_timer(duration, callback);
}

template<typename Class>
ros::Timer create_timer(
        double duration,
        void(Class::*callback)(const ros::TimerEvent&),
        Class* obj)
{
    return comm().create_timer<Class>(duration, callback, obj);
}

} // namespace ipc

} // namespace this_node