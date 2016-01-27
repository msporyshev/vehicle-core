#pragma once

#include <ros/ros.h>
#include <ros/message_traits.h>

#include <string>
#include <memory>
#include <functional>
#include <list>

namespace ipc {

template<typename Msg>
std::string classname(const Msg& msg) 
{
    std::string package_type_name = ros::message_traits::datatype(msg);
    std::string classname = package_type_name.substr(package_type_name.find("/") + 1, std::string::npos);
    return classname;
}

/**
Возвращает тайстэмп сообщения в секундах, если он есть.
Если таймстэмпа нет, то возвращает 0
*/
template<typename Msg>
double timestamp(const Msg& msg) 
{
    if (!ros::message_traits::hasHeader<Msg>()) {
        ROS_INFO_STREAM("There are no timestamp in " << classname(msg));
        return 0.0;
    }

    return ros::message_traits::timeStamp(msg)->toSec();
}

/**
Возвращает true, если разница времени отправки сообщения msg и текущего времени
меньше, чем timeout.
False в противном случае. 
*/
template<typename Msg>
bool is_actual(const Msg& msg, double timeout) {
    return ros::Time::now().toSec() - timestamp(msg) > timeout ? false : true;
}

template<typename Msg>
std::string topic_name(std::string module) {
    return "/" + module + "/" + classname(Msg());
}

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

    Subscriber() {}

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
        return receiver_->received;
    }

    const Msg& msg() const
    {
        return receiver_->msg;
    }

    const Msg& msg_wait() const
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
        void on_receive(const Msg& msg)
        {
            received = true;
            this->msg = msg;
            for (auto callback : callbacks) {
                callback(msg);
            }
        }

        bool received = false;
        Msg msg;
        std::list<Callback> callbacks;
    };

    std::shared_ptr<Receiver> receiver_;
    ros::Subscriber sub_;
};

class Communicator
{
public:
    Communicator(std::string package = ""): package_name_(package) {}

    const ros::NodeHandle& get_node() { return node_; }

    template<typename Msg>
    Subscriber<Msg> subscribe(std::string module, int queue_size = QUEUE_SIZE)
    {
        auto sub = std::make_shared<Subscriber<Msg> >(node_, topic_name<Msg>(module), queue_size);
        subscribers_.push_back(sub);
        return *sub;
    }

    template<typename Msg>
    Subscriber<Msg> subscribe(std::string module, void(*callback)(const Msg&), int queue_size = QUEUE_SIZE)
    {
        auto sub = std::make_shared<Subscriber<Msg> >(node_, topic_name<Msg>(module), callback, queue_size);
        subscribers_.push_back(sub);
        return *sub;
    }

    template<typename Class, typename Msg>
    Subscriber<Msg> subscribe(std::string module,
            void(Class::*callback)(const Msg&),
            Class* obj,
            int queue_size = QUEUE_SIZE)
    {
        auto sub = std::make_shared<Subscriber<Msg> >(node_, topic_name<Msg>(module), callback, obj, queue_size);
        subscribers_.push_back(sub);
        return *sub;
    }

    template<typename Msg>
    Subscriber<Msg> subscribe_cmd()
    {
        return subscribe<Msg>(package_name_);
    }

    template<typename Msg>
    Subscriber<Msg> subscribe_cmd(void(*callback)(const Msg&), int queue_size = QUEUE_SIZE)
    {
        return subscribe(package_name_, callback);
    }

    template<typename Class, typename Msg>
    Subscriber<Msg> subscribe_cmd(
            void(Class::*callback)(const Msg&),
            Class* obj,
            int queue_size = QUEUE_SIZE)
    {
        return subscribe(package_name_, callback, obj);
    }

    template<typename Msg>
    ros::Publisher advertise_cmd(std::string module, int queue_size = QUEUE_SIZE)
    {
        return node_.advertise<Msg>(topic_name<Msg>(module), queue_size);
    }

    template<typename Msg>
    ros::Publisher advertise(int queue_size = QUEUE_SIZE)
    {
        return node_.advertise<Msg>(topic_name<Msg>(package_name_), queue_size);
    }

    template<typename Msg>
    ros::Publisher advertise(std::string topic, int queue_size = QUEUE_SIZE)
    {
        return node_.advertise<Msg>(topic, queue_size);
    }

    ros::Timer create_timer(double duration, void(*callback)(const ros::TimerEvent&))
    {
        auto timer = node_.createTimer(ros::Duration(duration), callback);
        timers_.push_back(timer);
        return timer;
    }

    template<typename Class>
    ros::Timer create_timer(
            double duration,
            void(Class::*callback)(const ros::TimerEvent&),
            Class* obj)
    {
        auto timer = node_.createTimer(ros::Duration(duration), boost::bind(callback, obj, _1));
        timers_.push_back(timer);
        return timer;
    }


private:
    static const int QUEUE_SIZE;

    ros::NodeHandle node_;
    std::string package_name_;
    std::list<std::shared_ptr<SubscriberBase> > subscribers_;
    std::list<ros::Timer> timers_;
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