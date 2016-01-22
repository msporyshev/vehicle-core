#include "local_position_estimator.h"
#include "exceptions.h"

#include <cmath>
#include <algorithm>

#include <iostream>

const std::string LocalPositionEstimator::NODE_NAME = "local_position_estimator";

LocalPositionEstimator::LocalPositionEstimator(Device device) :
device_(device),
position_(MakePoint2(0., 0.)),
device_not_respond_(false),
last_device_time_(ros::Time::now()),
measurement_prev_(0., 0., 0., 0.)
{

}

void LocalPositionEstimator::init_ipc(ipc::Communicator& communicator)
{
    position_pub_ = communicator.advertise<navig::MsgEstimatedPosition>();

    imu_msg_ = communicator.subscribe<compass::MsgCompassAcceleration>("compass");
    imu_angle_ = communicator.subscribe<compass::MsgCompassAngle>("compass");
    dvl_msg_ = communicator.subscribe<dvl::MsgDvlVelocity>("dvl");
    /* Здесь нужно дописать подписку на сообщение о текущей скорости от регуляторов */
}

bool LocalPositionEstimator::current_device_ready()
{
    if ((ros::Time::now() - last_device_time_).toSec() > timeout_device_silence_) {
        if (device_not_respond_) {
            throw DeviceNotRespondException();
        } 
        else {
            device_not_respond_ = true;
            auto prev_device = device_;
            auto device = get_another_device();
            set_device(device);
            flush_position();
            ROS_INFO_STREAM("Device " << device_to_string(prev_device) << " is not respond. Switched to another device "
                << device_to_string(device) << ".");
        }
    }

    return device_ == Device::IMU ? imu_msg_.ready() : (dvl_msg_.ready() && imu_angle_.ready());
}

void LocalPositionEstimator::read_current_device_msg()
{
    navig::MsgEstimatedPosition current_position;
    if (device_ == Device::IMU) {
        try {
            current_position = calc_imu_position();
        }
        catch (std::exception& e) {
            ROS_INFO_STREAM(device_to_string(device_) << ": " << e.what());
            return;      
        }
    } 
    else {
        try {
            current_position = calc_dvl_position();
        }
        catch (std::exception& e) {
            ROS_INFO_STREAM(device_to_string(device_) << ": " << e.what());
            return;
        }
    }
    position_ = MakePoint2(current_position.x, current_position.y);

    last_device_time_ = ros::Time::now();

    position_pub_.publish(current_position);
}

int LocalPositionEstimator::get_period() const
{
    return delta_t_;
}

void LocalPositionEstimator::set_device(Device device)
{
    device_ = device;
}

libauv::Point2d LocalPositionEstimator::get_position() const
{
    return position_;
}

libauv::Point2d LocalPositionEstimator::flush_position()
{
    position_ = MakePoint2(0., 0.);
    measurement_prev_ = DynamicParameters(0., 0., 0., 0.);
    last_device_time_ = ros::Time::now();
}

void LocalPositionEstimator::read_config(navig::LocalPositionEstimatorConfig& config, unsigned int level)
{
    timeout_device_silence_ = config.timeout_device_silence;
    use_velocity_from_regul_ = config.use_velocity_from_regul;
    timeout_old_data_ = config.timeout_old_data;
    delta_t_ = config.delta_t;
}

navig::MsgEstimatedPosition LocalPositionEstimator::calc_imu_position()
{
    auto msg = *imu_msg_.msg();

    double duration = ros::Time::now().toSec() - ipc::timestamp(msg); 
    if (duration > timeout_old_data_) {
        throw OldDataException(duration);
    }

    auto& m = measurement_prev_;

    if (m.t == 0) {
        m.t = ros::Time::now().toSec();
    }

    double cur_msg_time = ipc::timestamp(msg);
    double cur_delta_t = cur_msg_time - m.t;
    double cur_vx = msg.acc_x * cur_delta_t + m.vx;
    double cur_vy = msg.acc_y * cur_delta_t + m.vy;
    double x = cur_delta_t * cur_vx;
    double y = cur_delta_t * cur_vy;

    m = DynamicParameters(cur_delta_t, cur_msg_time, msg.acc_x, msg.acc_y, cur_vx, cur_vy);
    
    navig::MsgEstimatedPosition position;
    position.x = position_.x + x;
    position.y = position_.y + y;

    ROS_INFO_STREAM("IMU position: " << position);
    return position;
}

navig::MsgEstimatedPosition LocalPositionEstimator::calc_dvl_position()
{

    auto angles = *imu_angle_.msg();
    auto velocity = *dvl_msg_.msg();

    double duration_max = std::max(ros::Time::now().toSec() - ipc::timestamp(velocity),
        ros::Time::now().toSec() - ipc::timestamp(angles));
    if (duration_max > timeout_old_data_) {
        throw OldDataException(duration_max);
    }

    double vel_north = velocity.velocity_forward * cos(angles.heading) 
        - velocity.velocity_right * sin(angles.heading);
    double vel_east = velocity.velocity_forward * sin(angles.heading) 
        + velocity.velocity_right * cos(angles.heading);
    
    double delta_t = get_delta_t();

    double delta_n = delta_t * vel_north;
    double delta_e = delta_t * vel_east;

    navig::MsgEstimatedPosition position;
    position.x = position_.x + delta_n;
    position.y = position_.y + delta_e;

    ROS_INFO_STREAM("DVL position: " << position);

    return position;
}

std::string LocalPositionEstimator::device_to_string(Device device)
{
    return device == Device::IMU ? "IMU" : "DVL";
}

LocalPositionEstimator::Device LocalPositionEstimator::get_another_device()
{
    if (device_ == Device::IMU) {
        return Device::DVL;
    }
    else {
        return Device::IMU;
    }
}

double LocalPositionEstimator::get_delta_t()
{
    auto& m = measurement_prev_;
    double cur_t = ros::Time::now().toSec();
    if (m.t == 0) {
        m.t = cur_t;
    }
    double delta_t = cur_t - m.t;
    m.t = cur_t;

    return delta_t;
}