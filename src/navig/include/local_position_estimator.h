#pragma once

#include <libauv/point/point.h>
#include <libipc/ipc.h>

#include <navig/LocalPositionEstimatorConfig.h>

#include <dvl/MsgDvlVelocity.h>
#include <compass/MsgCompassAcceleration.h>
#include <navig/MsgEstimatedPosition.h>

#include "dynamic_parameters.h"

#include <string>

class LocalPositionEstimator
{
public:
    enum Device { IMU, DVL };

    static const std::string NODE_NAME;
    
    /**
    В конструкторе указывается устройство, по данным которого будет считаться
    положение аппарата. По умолчанию это устройство - DVL
    */
    LocalPositionEstimator(Device device = Device::DVL);

    /**
    Инициализация обмена сообщениями у модуля.
    Здесь происходит подписка и регистрация для получения и отправки сообщений соответственно
    */
    void init_ipc(ipc::Communicator& communicator);

    bool current_device_ready() const;

    void read_current_device_msg();

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

    void read_config(navig::LocalPositionEstimatorConfig& config, unsigned int level);

    /**
    Изменяет имя конфигурационного файла и считывает новые параметры
    */
    // void change_config_params(const std::string& config_filename);

private:
    navig::MsgEstimatedPosition calc_imu_position();
    navig::MsgEstimatedPosition calc_dvl_position();

    std::string device_to_string();

    Device device_; ///> Устройство, от которого получаются измерения
    libauv::Point2d position_; ///> Текущая позиция аппарата в локальной системе координат, связанной с аппаратом [м]
    bool device_not_respond_; ///> Флаг, отвечает ли устройство
    double last_device_time_; ///> Последний раз, когда были получены данные от устройства
    
    ipc::Subscriber<compass::MsgCompassAcceleration> imu_msg_;
    ipc::Subscriber<dvl::MsgDvlVelocity> dvl_msg_;

    ros::Publisher position_pub_; 

    DynamicParameters measurement_prev_; ///> Данные предыдущего измерения

    double timeout_device_silence_; ///> Таймаут, после которого считается, что устройство не отвечает
    double timeout_old_data_; ///> Таймаут, после которого данные считаются устаревшими [c]
    int use_velocity_from_regul_; ///> Флаг, использовать ли данные о скорости от регуляторов
    int delta_t_; ///> Период чтения сообщений из топика [Hz]
};