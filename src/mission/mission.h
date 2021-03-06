#pragma once

/**
\file
\brief Заголовочный файл миссии

В данном файле находятся обработчики сообщений, принимаемых миссией,
а также методы, выполняющие обработку и публикацию сообщений миссии

*/

#include "task.h"

#include <mission/CmdStopMission.h>
#include <mission/CmdStartMission.h>
#include <mission/CmdAddTask.h>

#include <libipc/ipc.h>
#include <motion/motion_client/robosub_motion_client.h>
#include <config_reader/yaml_reader.h>

#include <memory>
#include <vector>
#include <string>
#include <set>
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
    ~Mission();

    bool run();
    void push_default_tasks();
    void push_tasks_branch(std::string branch);

    void push_task_to_progress(std::string task_name, YamlReader reader);
    Kitty process_next_task();

    void handle_stop_mission(const mission::CmdStopMission& msg);
    void handle_add_task(const mission::CmdAddTask& msg);
    void handle_start_mission(const mission::CmdStartMission& msg);
private:
    std::queue<TaskConfig> tasks_in_progress_;

    ipc::Communicator& communicator_;
    RobosubMotionClient motion_;
    YamlReader cfg_;

    ipc::Subscriber<mission::CmdAddTask> add_task_sub_;
    ipc::Subscriber<mission::CmdStartMission> start_mission_sub_;
    ipc::Subscriber<mission::CmdStopMission> stop_mission_sub_;

    std::set<std::string> prev_branches_;

    AUTOPARAM_OPTIONAL(bool, stop_after_fail_, false);
};