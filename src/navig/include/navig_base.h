#pragma once

#include <libipc/ipc.h>

#include <navig/NavigConfig.h>

#include <string>

class NavigBase
{
public:
    virtual std::string get_name() const = 0;
    
    void read_config(navig::NavigConfig& config, unsigned int level)
    {
        timeout_old_data_ = config.timeout_old_data;
        delta_t_ = config.delta_t;
    }

    virtual void init_ipc(ipc::Communicator& communicator) = 0;

    virtual void run() = 0;

protected:
    /**
    Шаблонный обработчик сообщений.
    Печатает на консоль тип полученного сообщения и его содержимое
    \param[in] msg Сообщение
    */
    template<typename MsgType>
    void handle_message(const MsgType& msg)
    {
        ROS_INFO_STREAM("Received " << ipc::classname(msg));
    }

    double timeout_old_data_;
    int delta_t_;

    ros::Publisher acc_pub_;
    ros::Publisher angles_pub_;
    ros::Publisher depth_pub_;
    ros::Publisher height_pub_;
    ros::Publisher position_pub_;
    ros::Publisher rates_pub_;
    ros::Publisher velocity_pub_;
};