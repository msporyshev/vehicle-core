#include "mission.h"

#include <cstdlib>
#include <ctime>

#include <navig/MsgNavigAccelerations.h>
#include <navig/MsgNavigAngles.h>
#include <navig/MsgNavigDepth.h>
#include <navig/MsgNavigHeight.h>
#include <navig/MsgNavigPosition.h>
#include <navig/MsgNavigRates.h>
#include <navig/MsgNavigVelocity.h>

#include "state_machine.h"
#include "task_factory.h"

#include <video/MsgFoundBin.h>

using namespace std;

const string Mission::NODE_NAME = "mission";

const string PACKAGE_NAME = "mission";

Mission::Mission(ipc::Communicator& comm)
        : communicator_(comm)
        , motion_(RobosubMotionClient(comm))
        , cfg_("mission.yml", PACKAGE_NAME)
{
    YAML::Node tasks_configs;
    ROS_INFO("Reading mission config...");

    cfg_.read_param(tasks_configs, "tasks");

    if (!tasks_configs.IsDefined()) {
        ROS_INFO("tasks field could not be read");
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

        reader.add_source(reader.to_filename(task_name));
        reader.add_source("task.yml");

        tasks_.push_back(RegisteredTasks::instance().init(task_name, reader, comm));
        names_.push_back(task_name);
    }
}

void Mission::run()
{
    for (int i = 0; i < tasks_.size(); i++) {
        std::string task_sign = "#" + to_string(i) + " (" + names_[i] + ")";

        ROS_INFO_STREAM("Starting task " << task_sign << " ...");

        double start_time = fixate_time();
        Kitty result = tasks_[i]->run();

        ROS_INFO_STREAM("Task " << task_sign << " has been finished in " << fixate_time() - start_time);

        if ((result == Kitty::Sad || result == Kitty::Angry) && stop_after_fail_.get()) {
            ROS_INFO("Task has been failed, stopping mission");
            break;
        }
    }
}

int main(int argc, char* argv[])
{
    auto communicator = ipc::init(argc, argv, Mission::NODE_NAME);
    Mission mission(communicator);

    ipc::EventLoop loop(10);
    mission.run();
    return 0;
}