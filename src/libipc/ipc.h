#pragma once

#include <ros/ros.h>

#include <string>
#include <memory>
#include <functional>
#include <list>

namespace ipc {

class SubscriberBase
{
public:
    virtual bool ready() const = 0;
    virtual void wait_ready() const = 0;
};

template<typename Msg>
class Subscriber: public SubscriberBase
{
public:
    using Callback = std::function<void(const Msg&)>;

    Subscriber(ros::NodeHandle& node, std::string topic, int queue_size)
    {
        receiver_ = std::make_shared<Receiver>();

        sub_ = node.subscribe(topic, queue_size, &Receiver::on_receive, receiver_.get());
    }

    Subscriber(ros::NodeHandle& node,
            std::string topic,
            Callback callback,
            int queue_size): Subscriber(node, topic, queue_size) // don't touch it without c++11 !!!
    {
        receiver_->callbacks.push_back(callback);
    }


    template<typename Class>
    Subscriber(ros::NodeHandle& node,
            std::string topic,
            void(Class::*callback)(const Msg&),
            Class* obj,
            int queue_size): Subscriber(node, topic, std::bind(callback, obj, std::placeholders::_1), queue_size)
    {

    }

    virtual bool ready() const override
    {
        return receiver_->msg.get() != nullptr;
    }

    boost::shared_ptr<Msg const> msg() const
    {
        return receiver_->msg;
    }

    const Msg& msg_wait() const
    {
        wait_ready();
        return *receiver_->msg;
    }

    virtual void wait_ready() const override
    {
        ros::Rate rate(10);
        while (!ready() && ros::ok()) {
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    class Receiver
    {
    public:
        void on_receive(const boost::shared_ptr<Msg const>& msg)
        {
            this->msg = msg;
            for (auto callback : callbacks) {
                callback(*msg);
            }
        }

        boost::shared_ptr<Msg const> msg;
        std::list<Callback> callbacks;
    };

    std::shared_ptr<Receiver> receiver_;
    ros::Subscriber sub_;
};

class Communicator
{
public:
    Communicator(std::string package = ""): package_name_(package) {}

    template<typename Msg>
    Subscriber<Msg> subscribe(std::string module, int queue_size = QUEUE_SIZE)
    {
        auto sub = std::make_shared<Subscriber<Msg> >(node_, "/" + module, queue_size);
        subscribers_.push_back(sub);
        return *sub;
    }

    template<typename Msg>
    Subscriber<Msg> subscribe(std::string module, void(*callback)(const Msg&), int queue_size = QUEUE_SIZE)
    {
        auto sub = std::make_shared<Subscriber<Msg> >(node_, "/" + module, callback, queue_size);
        subscribers_.push_back(sub);
        return *sub;
    }

    template<typename Class, typename Msg>
    Subscriber<Msg> subscribe(std::string module,
            void(Class::*callback)(const Msg&),
            Class* obj,
            int queue_size = QUEUE_SIZE)
    {
        auto sub = std::make_shared<Subscriber<Msg> >(node_, "/" + module, callback, obj, queue_size);
        subscribers_.push_back(sub);
        return *sub;
    }

    template<typename Msg>
    Subscriber<Msg> subscribe_cmd()
    {
        return subscribe<Msg>("/" + package_name_ + CMD_SUFFIX);
    }

    template<typename Msg>
    Subscriber<Msg> subscribe_cmd(void(*callback)(const Msg&), int queue_size = QUEUE_SIZE)
    {
        return subscribe("/" + package_name_ + CMD_SUFFIX, callback);
    }

    template<typename Class, typename Msg>
    Subscriber<Msg> subscribe_cmd(
            void(Class::*callback)(const Msg&),
            Class* obj,
            int queue_size = QUEUE_SIZE)
    {
        return subscribe("/" + package_name_ + CMD_SUFFIX, callback, obj);
    }

    template<typename Msg>
    ros::Publisher advertise_cmd(std::string module, int queue_size = QUEUE_SIZE)
    {
        return node_.advertise<Msg>("/" + module + CMD_SUFFIX, queue_size);
    }

    template<typename Msg>
    ros::Publisher advertise(int queue_size = QUEUE_SIZE)
    {
        return node_.advertise<Msg>(package_name_, queue_size);
    }

    template<typename Msg>
    ros::Publisher advertise(std::string topic, int queue_size = QUEUE_SIZE)
    {
        return node_.advertise<Msg>(topic, queue_size);
    }

private:
    static const std::string CMD_SUFFIX;
    static const int QUEUE_SIZE;

    ros::NodeHandle node_;
    std::string package_name_;
    std::list<std::shared_ptr<SubscriberBase> > subscribers_;
};

class EventLoop
{
public:
    EventLoop(int rate): rate_(rate) {}

    bool ok()
    {
        ros::spinOnce();
        rate_.sleep();
        return ros::ok();
    }

private:
    ros::Rate rate_;
};

Communicator init(int argc, char** argv, std::string node_name);

} // namespace ipc