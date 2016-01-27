#pragma once

#include <libauv/point/point.h>
#include <libipc/ipc.h>

#include <navig/PositionEstimatorConfig.h>

#include <dvl/MsgDvlVelocity.h>
#include <compass/MsgCompassAcceleration.h>
#include <compass/MsgCompassAngle.h>
#include <navig/MsgEstimatedPosition.h>

#include "dynamic_parameters.h"

#include <string>

class PositionEstimator
{
public:
    enum Device { IMU, DVL };

    static const std::string NODE_NAME;
    
    /**
    В конструкторе указывается устройство, по данным которого будет считаться
    положение аппарата. По умолчанию это устройство - DVL
    */
    PositionEstimator(Device device = Device::DVL);

    /**
    Инициализация обмена сообщениями у модуля.
    Здесь происходит подписка и регистрация для получения и отправки сообщений соответственно
    */
    void init_ipc(ipc::Communicator& communicator);

    /**
    Возвращает готовность нового сообщения у текущего устройства device
    */
    bool current_device_ready();

    /**
    Считывает последнее сообщение у текущего устройства device
    */
    void read_current_device_msg();

    /**
    Возвращает период в герцах, с которым проверяется наличие сообщений от устройства device
    */
    int get_period() const;

    /**
    Изменяет устройство, по данным которого считается положение аппарата
    */
    void set_device(Device device);

    /**
    Возвращает последнюю расчитанную позицию аппарата
    */
    libauv::Point2d get_position() const;

    /**
    Возвращает последнюю расчитанную позицию аппарата, сбрасывает точку старта
    и устанавливает текущее положение в качестве точки старта
    */
    libauv::Point2d flush_position();

    /**
    Это колбэк. Он получает уже считанный конфигурационный файл от росовского сервера конфигов
    */
    void read_config(navig::PositionEstimatorConfig& config, unsigned int level);

private:
    /**
    Вычисляет текущее положение, основываясь на данных от IMU
    */
    navig::MsgEstimatedPosition calc_imu_position();
    
    /**
    Вычисляет текущее положение, основываясь на данных от DVL
    */
    navig::MsgEstimatedPosition calc_dvl_position();

    /**
    Конвертирует значение переменной типа Device в std::string
    */
    std::string device_to_string(Device device);

    /**
    Возвращает разницу во времени между ros::Time::now() и measurement_prev_.t
    или 0, если measurement_prev_.t == 0
    */
    template<typename MsgType>
    double get_delta_t(const MsgType& msg)
    {
        auto& m = measurement_prev_;
        double cur_t = ipc::timestamp(msg);
        if (m.t == 0) {
            m.t = cur_t;
        }
        double delta_t = cur_t - m.t;
        m.t = cur_t;

        return delta_t;
    }

    double calc_vel_north(double vel_f, double vel_r, double heading);
    double calc_vel_east(double vel_f, double vel_r, double heading);
    
    Device get_another_device();

    Device device_; ///> Устройство, от которого получаются измерения
    libauv::Point2d position_; ///> Текущая позиция аппарата в локальной системе координат, связанной с аппаратом [м]
    bool device_not_respond_; ///> Флаг, отвечает ли устройство
    ros::Time last_device_time_; ///> Последний раз, когда были получены данные от устройства
    
    ipc::Subscriber<compass::MsgCompassAcceleration> imu_msg_; ///> Для чтения сообщений об ускорениях от IMU
    ipc::Subscriber<compass::MsgCompassAngle> imu_angle_; ///> Для чтения сообщений об углах от IMU
    ipc::Subscriber<dvl::MsgDvlVelocity> dvl_msg_; ///> Для чтения сообщений о скорости от DVL

    ros::Publisher position_pub_; ///> Для публикации сообщений о текущем местоположении

    DynamicParameters measurement_prev_; ///> Данные предыдущего измерения

    double timeout_device_silence_; ///> Таймаут, после которого считается, что устройство не отвечает
    double timeout_old_data_; ///> Таймаут, после которого данные считаются устаревшими [c]
    int use_velocity_from_regul_; ///> Флаг, использовать ли данные о скорости от регуляторов
    int delta_t_; ///> Период чтения сообщений из топика [Hz]
};