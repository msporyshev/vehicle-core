/**
\file
\brief Заголовочный файл навигационного модуля

В данном файле находятся обработчики сообщений, принимаемых навигом,
а также методы, выполняющие обработку и публикацию сообщений навигационного модуля

\ingroup navig_node
*/

///@{
#pragma once

#include <string>

#include <ros/ros.h>

#include <compass/MsgCompassAngle.h>
#include <compass/MsgCompassAcceleration.h>
#include <compass/MsgCompassAngleRate.h>
#include <navig/MsgEstimatedPosition.h>
#include <dvl/MsgDvlDistance.h>
#include <dvl/MsgDvlVelocity.h>
#include <dvl/MsgDvlHeight.h>
#include <gps/MsgGpsCoordinate.h>
#include <gps/MsgGpsSatellites.h>
#include <gps/MsgGpsUtc.h>
#include <supervisor/MsgSupervisorDepth.h>

#include <libipc/ipc.h>

#include <navig/NavigConfig.h>

namespace navig
{
    static const std::string NODE_NAME = "Navig";

    /**
    Метод выполняет подписку на все сообщения, 
    принимаемые навигом и регистрирует все сообщения,
    публикуемые навигом
    */
    void init_ipc(ipc::Communicator& communicator);

    /**
    Шаблонный обработчик сообщений.
    Печатает на консоль тип полученного сообщения и его содержимое
    \param[in] msg Сообщение
    */
    template<typename T>
    void handle_message(const T& msg)
    {
        ROS_INFO_STREAM("Received " << ipc::classname(msg));
    }
    
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

    void read_config(navig::NavigConfig& config, unsigned int level);

    int get_period();

    /**
    Функция для моделирования
    Создает и публикует сообщения об ускорениях.
    */
    void create_and_publish_acc();
    
    /**
    Функция для моделирования
    Создает и публикует сообщения об углах (курс, крен, дифферент)
    */
    void create_and_publish_angles();

    /**
    Функция для моделирования
    Создает и публикует сообщения об угловых ускорениях
    */
    void create_and_publish_rates();
    
    /**
    Функция для моделирования
    Создает и публикует сообщения о глубине
    */
    void create_and_publish_depth();
    
    /**
    Функция для моделирования
    Создает и публикует сообщения о высоте над дном
    */
    void create_and_publish_height();

    /**
    Функция для моделирования
    Создает и публикует сообщения о местоположении в локальной и глобальной
    системах координат
    */
    void create_and_publish_position();

    /**
    Функция для моделирования
    Создает и публикует сообщения о скоростях аппарата
    */
    void create_and_publish_velocity();
}

///@}