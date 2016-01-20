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
    std::string get_name() const;

    /**
    Метод выполняет подписку на все сообщения, 
    принимаемые навигом и регистрирует все сообщения,
    публикуемые навигом
    */
    void init_ipc(ipc::Communicator& communicator);

    void run();

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

    int get_period() const;

private:
    std::string NODE_NAME = "navig";

    std::map<std::string, double> old_time_ = {
        { ipc::classname(compass::MsgCompassAngle()), 0.0 },
        { ipc::classname(compass::MsgCompassAcceleration()), 0.0 },
        { ipc::classname(compass::MsgCompassAngleRate()), 0.0 },
        { ipc::classname(navig::MsgEstimatedPosition()), 0.0 },
        { ipc::classname(dvl::MsgDvlDistance()), 0.0 },
        { ipc::classname(dvl::MsgDvlVelocity()), 0.0 },
        { ipc::classname(dvl::MsgDvlHeight()), 0.0 },
        { ipc::classname(gps::MsgGpsCoordinate()), 0.0 },
        { ipc::classname(gps::MsgGpsSatellites()), 0.0 },
        { ipc::classname(gps::MsgGpsUtc()), 0.0 },
        { ipc::classname(supervisor::MsgSupervisorDepth()), 0.0 }
    };

    ros::Publisher acc_pub_;
    ros::Publisher angles_pub_;
    ros::Publisher depth_pub_;
    ros::Publisher height_pub_;
    ros::Publisher position_pub_;
    ros::Publisher rates_pub_;
    ros::Publisher velocity_pub_;

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

    double old_depth_time_ = 0.0;
    double old_depth_ = 0;

    navig::MsgNavigVelocity velocity_data_;
    navig::MsgNavigAngles angles_data_;

    template<typename MsgType>
    bool check_time(const MsgType& msg)
    {
        bool result(true);
    
        auto new_time = ipc::timestamp(msg);
        if (old_time_[ipc::classname(msg)] != 0.0) { 
            result = new_time - old_time_[ipc::classname(msg)] <= timeout_old_data_;
        }
        old_time_[ipc::classname(msg)] = new_time;
    
        return result;
    }

    double calc_depth_velocity(double depth, double time_diff);
    void send_velocity();
};

///@}