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
            int queue_size,
            std::function<void(const Msg&)> callback): Subscriber(node, topic, queue_size) // don't touch it without c++11 !!!
    {
        receiver_->callbacks.push_back(callback);
    }


    template<typename Class>
    Subscriber(ros::NodeHandle& node,
            std::string topic,
            void(Class::*callback)(const Msg&),
            Class* obj,
            int queue_size = 1000): Subscriber(node, topic, queue_size, std::bind(callback, obj, std::placeholders::_1))
    {

    }

    virtual bool ready() const override
    {
        return receiver_->msg != nullptr;
    }

    boost::shared_ptr<Msg const> msg() const
    {
        return receiver_->msg;
    }

    boost::shared_ptr<Msg const> msg_wait() const
    {
        wait_ready();
        return receiver_->msg;
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

        boost::shared_ptr<Msg const> msg = nullptr;
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
    Subscriber<Msg> subscribe(std::string topic, int queue_size = 1000)
    {
        auto sub = std::make_shared<Subscriber<Msg> >(node_, topic, queue_size);
        subscribers_.push_back(sub);
        return *sub;
    }

    template<typename Msg>
    Subscriber<Msg> subscribe(std::string topic, std::function<void(const Msg&)> callback, int queue_size = 1000)
    {
        auto sub = std::make_shared<Subscriber<Msg> >(node_, topic, callback, queue_size);
        subscribers_.push_back(sub);
        return *sub;
    }

    template<typename Class, typename Msg>
    Subscriber<Msg> subscribe (std::string topic,
            void(Class::*callback)(const Msg&),
            Class* obj,
            int queue_size = 1000)
    {
        auto sub = std::make_shared<Subscriber<Msg> >(node_, topic, callback, obj, queue_size);
        subscribers_.push_back(sub);
        return *sub;
    }

private:
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