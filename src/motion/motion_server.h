#pragma once

/**
\file
\brief Заголовочный файл регуляторов

В данном файле находятся обработчики сообщений, принимаемых регуляторами,
а также методы, выполняющие обработку и публикацию сообщений регуляторов

*/

#include <string>
#include <iostream>

#include <ros/ros.h>

#include <libipc/ipc.h>

class MotionServer
{
public:
    MotionServer(ipc::Communicator& communicator);
    ~MotionServer();

    /**
    Метод выполняет подписку на все сообщения, 
    принимаемые регуляторами и регистрирует все сообщения,
    публикуемые регуляторами
    */
    void init_ipc();

    /**
    Создает и публикует сообщения о статусе команды
    */
    void create_and_publish_cmd_status();

    /**
    Создает и публикует сообщения к БУДам
    */
    void create_and_publish_regul();

    /**
    Шаблонный обработчик сообщений.
    Печатает на консоль тип полученного сообщения и его содержимое
    \param[in] msg Сообщение
    */
    template<typename T>
    void handle_command(const T& msg)
    {
        ROS_INFO("Message %s received", ros::message_traits::datatype<T>());
        std::cout << msg << std::endl;
    }

    ///< Имя модуля
    static const std::string NODE_NAME;

private:
    ipc::Communicator& communicator_;
    ros::Publisher cmd_status_pub_, regul_pub_;
};
