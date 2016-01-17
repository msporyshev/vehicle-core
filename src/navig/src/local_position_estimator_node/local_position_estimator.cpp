#include "local_position_estimator.h"
#include "too_old_data_exception.h"

#include <iostream>

const std::string LocalPositionEstimator::NODE_NAME = "Local_position_estimator";

LocalPositionEstimator::LocalPositionEstimator(Device device) :
device_(device),
position_(MakePoint2(0., 0.)),
device_not_respond_(false),
last_device_time_(0.),
measurement_prev_(0., 0., 0., 0.)
{

}

void LocalPositionEstimator::init_ipc(ipc::Communicator& communicator)
{
    position_pub_ = communicator.advertise<navig::MsgEstimatedPosition>();

    imu_msg_ = communicator.subscribe<compass::MsgCompassAcceleration>("compass");
    dvl_msg_ = communicator.subscribe<dvl::MsgDvlVelocity>("dvl");
    /* Здесь нужно дописать подписку на сообщение о текущей скорости от регуляторов */
}

bool LocalPositionEstimator::current_device_ready() const
{
    return device_ == Device::IMU ? imu_msg_.ready() : dvl_msg_.ready();
}

void LocalPositionEstimator::read_current_device_msg()
{
    navig::MsgEstimatedPosition current_position;
    if (device_ == Device::IMU) {
        try {
            current_position = calc_imu_position();
        }
        catch (TooOldData tod) {
            ROS_INFO_STREAM("Received too old data from device " << device_to_string() << ": " << tod.duration);
            current_position.x = 0;
            current_position.y = 0;       
        }
    } 
    else {
        current_position = calc_dvl_position();
    }
    position_ += MakePoint2(current_position.x, current_position.y);

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

    double duration = (ros::Time::now() - *ros::message_traits::timeStamp(msg)).toSec(); 
    if (duration > timeout_old_data_) {
        throw TooOldData(duration);
    }

    auto& m = measurement_prev_;
    double current_msg_time = ros::message_traits::timeStamp(msg)->toSec();
    double cur_delta_t = ros::message_traits::timeStamp(msg)->toSec() - m.t;
    double cur_vx = msg.acc_x - m.vx;
    double cur_vy = msg.acc_y - m.vy;
    double x = current_msg_time * (m.ax * m.delta_t + msg.acc_x * cur_delta_t) - cur_vx;
    double y = current_msg_time * (m.ay * m.delta_t + msg.acc_y * cur_delta_t) - cur_vy;

    m = DynamicParameters(cur_delta_t, current_msg_time, msg.acc_x, msg.acc_y, cur_vx, cur_vy);
    
    navig::MsgEstimatedPosition position;
    position.x = x;
    position.y = y;

    return position;
}

navig::MsgEstimatedPosition LocalPositionEstimator::calc_dvl_position()
{
    //Это просто временная заглушка
    navig::MsgEstimatedPosition position;
    position.x = 0;
    position.y = 0;

    return position;
}

std::string LocalPositionEstimator::device_to_string()
{
    return device_ == Device::IMU ? "IMU" : "DVL";
}