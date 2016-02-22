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

    void run();

    ///< Имя модуля
    static const std::string NODE_NAME;
private:
    std::vector<ros::Publisher> publishers_;

    std::vector<std::shared_ptr<TaskBase> > tasks_;
    std::vector<std::string> names_;

    ipc::Communicator& communicator_;
    RobosubMotionClient motion_;
    YamlReader cfg_;

    AUTOPARAM_OPTIONAL(bool, stop_after_fail_, false);
};