/**
\file
\brief Заголовочный файл навигационного модуля

В данном файле находятся обработчики сообщений, принимаемых навигом,
а также методы, выполняющие обработку и публикацию сообщений навигационного модуля

\ingroup navig_node
*/

///@{
#pragma once

#include <navig_base.h>

#include <ros/ros.h>

#include <navig_message_list.h>

#include <navig/NavigConfig.h>
#include <navig/MsgNavigAngles.h>
#include <navig/MsgNavigVelocity.h>

class Navig : public NavigBase
{
public:
    std::string get_name() const override;

    /**
    Метод выполняет подписку на все сообщения, 
    принимаемые навигом и регистрирует все сообщения,
    публикуемые навигом
    */
    void init_ipc(ipc::Communicator& communicator) override;

    void run() override;

    /**
    Выполняет обработку и дальнейшую публикацию информации об углах от компаса
    \param[in] msg Углы (курс, крен, дифферент)
    */
    void handle_angles(const compass::MsgCompassAngle& msg);
    
    /**
    Выполняет обработку и дальнейшую публикацию информации об ускорениях от компаса
    \param[in] msg Ускорения
    */
    void handle_acceleration(const compass::MsgCompassAcceleration& msg);
    
    /**
    Выполняет обработку и дальнейшую публикацию информации об угловых ускорениях от компаса
    \param[in] msg Угловые ускорения
    */
    void handle_rate(const compass::MsgCompassAngleRate& msg);

    /**
    Выполняет обработку и дальнейшую публикацию информации о глубине от супервизора
    \param[in] msg Глубина
    */
    void handle_depth(const supervisor::MsgSupervisorDepth& msg);

    /**
    Выполняет обработку и дальнейшую публикацию информации о координатах от LocalPositionEstimator
    \param[in] msg Координаты
    */
    void handle_position(const navig::MsgEstimatedPosition& msg);

    void handle_velocity(const dvl::MsgDvlVelocity& msg);

private:
    std::string NODE_NAME = "navig";

    std::map<std::string, double> old_time_;

    ipc::Subscriber<compass::MsgCompassAngle> imu_angle_;
    ipc::Subscriber<compass::MsgCompassAcceleration> imu_acc_;
    ipc::Subscriber<compass::MsgCompassAngleRate> imu_rate_;
    ipc::Subscriber<navig::MsgEstimatedPosition> est_position_;
    ipc::Subscriber<dvl::MsgDvlDistance> dvl_dist_;
    ipc::Subscriber<dvl::MsgDvlVelocity> dvl_vel_;
    ipc::Subscriber<dvl::MsgDvlHeight> dvl_height_;
    ipc::Subscriber<gps::MsgGpsCoordinate> gps_coord_;
    ipc::Subscriber<gps::MsgGpsSatellites> gps_sat_;
    ipc::Subscriber<gps::MsgGpsUtc> gps_utc_;
    ipc::Subscriber<supervisor::MsgSupervisorDepth> supervisor_depth_;

    ros::Time start_time_ = ros::Time::now();

    double old_depth_time_ = 0.0;
    double old_depth_ = 0;

    navig::MsgNavigVelocity velocity_data_;
    navig::MsgNavigAngles angles_data_;

    /**
    Метод для чтения сообщений типа Msg в синхронном режиме и их обработка соответствующим хэндлером
    */
    template<typename Msg, typename Cls>
    void read_msg(ipc::Subscriber<Msg>& sub, void (Cls::*handle_msg)(const Msg&))
    {
        if (!sub.ready() && (ros::Time::now() - start_time_).toSec() > timeout_silence_) {
            ROS_INFO_STREAM("Message " << ipc::classname(sub.msg()) << " hasn't been receiving for " << timeout_silence_ << " seconds.");
        } else if (ipc::is_actual(sub.msg(), timeout_old_data_)) {
            (this->*handle_msg)(sub.msg());
        } else if (ipc::timestamp(sub.msg()) != 0.0) {
            ROS_INFO_STREAM("Message " << ipc::classname(sub.msg()) << " from navig hasn't been receiving for " << timeout_old_data_ << " seconds.");
        }
    }

    double calc_depth_velocity(double depth, double time_diff);
    void send_velocity();
};

///@}