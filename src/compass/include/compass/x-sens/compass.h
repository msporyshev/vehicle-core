#pragma once

#include "compass/x-sens/MTI.h"

#include <libipc/ipc.h>
#include "ros/ros.h"

#include "compass/MsgCompassAcceleration.h"
#include "compass/MsgCompassAngle.h"
#include "compass/MsgCompassAngleRate.h"
#include "compass/MsgCompassMagnetometer.h"

#include "compass/CmdConfig.h"
#include "compass/CmdDeclination.h"

#include <string>
#include <cmath>

#define _USE_MATH_DEFINES

enum class CommandCode {
  Init,                     // Инициализация компаса
  CalibrationOn,            // Запуск процесса калибровки компаса
  CalibrationOff,           // Остановка процесса калибровки компаса
  Reset,                    // Сброс компаса
  StoreFilter,              // Сохранение текущих значений встроенных фильтров
  CommandsSize
};

struct CompassConfig
{
    std::string port;
    int         baundrate;
    float       declination;
    bool        modelling;

    CompassConfig() {
        port = "";
        baundrate = 0;
        declination = 0;
        modelling = false;
    }
};

struct CompassCommand
{
    std::string command;
    void(MTI::*function)(int);
};

class Compass
{
    int com_handle_;
    
    // Структура с полученными от устройства данными измерений
    MTI_struct MTI_data_;

    // Флаг наличия актуальных данных от компаса в структуре MTI_data
    bool new_data_avalible_;

    MTI mti_;

    CompassConfig config_;

    std::vector<CompassCommand> compass_commands_;

    ros::Timer timer_data_update_;
    ros::Timer timer_data_publish_;
    ros::Timer timer_data_publish_model_;

    ros::Publisher acceleration_pub_;
    ros::Publisher angle_pub_;
    ros::Publisher angle_rate_pub_;
    ros::Publisher magnetometer_pub_;

    compass::MsgCompassAcceleration msg_acceleration_;
    compass::MsgCompassAngle msg_angle_;
    compass::MsgCompassAngleRate msg_angle_rate_;
    compass::MsgCompassMagnetometer msg_magnetometer_;
    void data_update(const ros::TimerEvent& event);
    void data_publish(const ros::TimerEvent& event);
    void data_publish_modelling(const ros::TimerEvent& event);

    void handle_message(const compass::CmdConfig& msg);
    void handle_message(const compass::CmdDeclination& msg);

public:

    const static std::string NODE_NAME;

	Compass(CompassConfig config);

    void init_connection(ipc::Communicator& communicator);
    void start_timers(ipc::Communicator& communicator);

    void init_mti();
    void close_mti();
};