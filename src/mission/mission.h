#pragma once

/**
\file
\brief Заголовочный файл миссии

В данном файле находятся обработчики сообщений, принимаемых миссией,
а также методы, выполняющие обработку и публикацию сообщений миссии

*/

#include "task.h"

#include <libipc/ipc.h>
#include <motion/motion_client/robosub_motion_client.h>
#include <config_reader/yaml_reader.h>

#include <memory>
#include <vector>
#include <string>

class Mission
{
public:
    Mission(ipc::Communicator& communicator);

    /**
    Метод выполняет подписку на все сообщения,
    принимаемые навигом и регистрирует все сообщения,
    публикуемые навигом
    */
    void init_ipc();

    /**
    Создание и публикация команд регуляторам
    */
    void publish_commands();

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

    void run();

    ///< Имя модуля
    static const std::string NODE_NAME;
private:
    std::vector<ros::Publisher> publishers_;

    std::vector<std::shared_ptr<TaskBase> > tasks_;

    ipc::Communicator& communicator_;
    RobosubMotionClient motion_;
    YamlReader cfg_;

    AUTOPARAM_OPTIONAL(bool, stop_after_fail_, false);
};