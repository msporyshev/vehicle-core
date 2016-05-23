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
#include <queue>

struct TaskConfig {
    std::string task_name;
    YamlReader config;

    TaskConfig(std::string task_name, YamlReader config): task_name(task_name), config(config) {}
};

class Mission
{
public:
    Mission(ipc::Communicator& communicator);

    void run();
    void push_default_tasks();
    void push_task_to_progress(std::string task_name, YamlReader reader);
    Kitty process_next_task();


    ///< Имя модуля
    static const std::string NODE_NAME;
private:
    std::vector<ros::Publisher> publishers_;

    std::queue<TaskConfig> tasks_in_progress_;

    ipc::Communicator& communicator_;
    RobosubMotionClient motion_;
    YamlReader cfg_;

    AUTOPARAM_OPTIONAL(bool, stop_after_fail_, false);
};