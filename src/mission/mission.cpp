#include "mission.h"

#include "state_machine.h"
#include "task_factory.h"

#include <string>


using namespace std;
using namespace mission;

const string PACKAGE_NAME = "mission";

Mission::Mission(ipc::Communicator& comm)
        : communicator_(comm)
        , motion_(RobosubMotionClient(comm))
        , cfg_("mission.yml", PACKAGE_NAME)
{
    add_task_sub_ = communicator_.subscribe_cmd(&Mission::handle_add_task, this);
    start_mission_sub_ = communicator_.subscribe_cmd(&Mission::handle_start_mission, this);
    stop_mission_sub_ = communicator_.subscribe_cmd(&Mission::handle_stop_mission, this);
}

Mission::~Mission()
{
    ROS_INFO("Mission finishing. Unfix all.");
    motion_.unfix_all();
}

void Mission::handle_stop_mission(const CmdStopMission& msg) {
    ROS_INFO("Received stop mission cmd, stopping.");

    while(!tasks_in_progress_.empty()) {
        tasks_in_progress_.pop();
    }
}

void Mission::handle_add_task(const CmdAddTask& msg) {
    ROS_INFO_STREAM("Received add task cmd: " << msg.task_name.data);
    auto config = YamlReader::from_string(msg.config.data);
    push_task_to_progress(msg.task_name.data, config);
}

void Mission::handle_start_mission(const CmdStartMission& msg) {
    ROS_INFO("Received start mission cmd");
    if (msg.tasks.empty()) {
        ROS_INFO("Tasks list is empty, starting default mission");

        push_default_tasks();
    } else {
        for (const auto& cmd: msg.tasks) {
            handle_add_task(cmd);
        }
    }
}

void Mission::push_tasks_branch(std::string branch) {
    YAML::Node tasks_configs;
    ROS_INFO("Reading mission config...");
    cfg_.read_param(tasks_configs, branch);

    if (!tasks_configs.IsDefined()) {
        ROS_INFO_STREAM("" << branch << " field could not be read");
        throw;
    }

    ROS_INFO("reading tasks configs...");
    for (size_t i = 0; i < tasks_configs.size(); ++i) {
        YamlReader reader;
        reader.set_package(PACKAGE_NAME);
        std::string task_name = tasks_configs[i]["name"].as<std::string>();

        ROS_INFO_STREAM("Reading " << task_name << " config...");

        try {
            reader.add_source(tasks_configs[i]["params"]);
        } catch(YAML::Exception e) {
            // Не делаем ничего, отсутствие поля params -- допустимая ситуация
        }

        push_task_to_progress(task_name, reader);
    }
}

void Mission::push_default_tasks() {
    push_tasks_branch("tasks");
}

void Mission::push_task_to_progress(string task_name, YamlReader reader) {
    reader.set_package(PACKAGE_NAME);
    reader.add_source(YamlReader::to_filename(task_name));
    reader.add_source("task.yml");

    tasks_in_progress_.emplace(task_name, reader);
}

Kitty Mission::process_next_task() {
    if (tasks_in_progress_.empty()) {
        ROS_INFO("Tried to start empty mission");
    }

    TaskConfig task_config = tasks_in_progress_.front();
    tasks_in_progress_.pop();

    auto current_task = RegisteredTasks::instance().create(
        task_config.task_name, task_config.config, communicator_);

    //TODO: change logging std::string task_sign = "#" + to_string(i) + " (" + task_config.task_name + ")";

    ROS_INFO_STREAM("Starting task " << task_config.task_name << " ...");

    double start_time = timestamp();
    current_task->prepare();
    if (task_config.task_name == "pinger_task") {
        if (last_branch_ == "octagon") {
            current_task->set_next_branch("bins");
        } else if (last_branch_ == "bins") {
            current_task->set_next_branch("octagon");
        }
    }

    Kitty result = current_task->run();

    if (current_task->next_branch() != "" && !prev_branches_.count(current_task->next_branch())) {
        prev_branches_.insert(current_task->next_branch());
        push_tasks_branch(current_task->next_branch());
    }

    ROS_INFO_STREAM("Task " << task_config.task_name << " has been finished in " << timestamp() - start_time);
    return result;
}

bool Mission::run()
{
    while(!tasks_in_progress_.empty()) {
        ros::spinOnce();

        if (!ros::ok()) {
            ROS_INFO("Ros communication failed, stopping mission");
            motion_.unfix_all();
            break;
        }

        Kitty result = process_next_task();

        if ((result == Kitty::Sad || result == Kitty::Angry) && stop_after_fail_.get()) {
            ROS_INFO("Task has been failed, stopping mission");
            motion_.unfix_all();
            break;
        }

        if (tasks_in_progress_.empty()) {
            motion_.unfix_all();
        }
    }

    while(!tasks_in_progress_.empty()) tasks_in_progress_.pop();

    return ros::ok();
}
