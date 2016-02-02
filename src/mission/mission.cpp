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

Mission::Mission(ipc::Communicator& comm)
        : communicator_(comm)
        , motion_(RobosubMotionClient(comm))
        , cfg_("mission.yml", "mission")
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
    }
}

void Mission::run()
{

}

void Mission::init_ipc()
{
    /**
        Это подписка на навиг
    */
    communicator_.subscribe("navig", &Mission::handle_message<navig::MsgNavigAccelerations>, this);
    communicator_.subscribe("navig", &Mission::handle_message<navig::MsgNavigAngles>, this);
    communicator_.subscribe("navig", &Mission::handle_message<navig::MsgNavigDepth>, this);
    communicator_.subscribe("navig", &Mission::handle_message<navig::MsgNavigHeight>, this);
    communicator_.subscribe("navig", &Mission::handle_message<navig::MsgNavigPosition>, this);
    communicator_.subscribe("navig", &Mission::handle_message<navig::MsgNavigRates>, this);
    communicator_.subscribe("navig", &Mission::handle_message<navig::MsgNavigVelocity>, this);

    communicator_.subscribe("video", &Mission::handle_message<video::MsgFoundBin>, this);
    /**
        Вот здесь нужно будет вписать подписку на сообщения от модуля видео
    */
}

void Mission::publish_commands()
{
    /**
        Публикация рандомных команд от миссии регуляторам
    */
    motion_.fix_heading(static_cast<float>(rand()) / static_cast<float>(RAND_MAX / 360), rand(), WaitMode::DONT_WAIT);
    motion_.fix_depth(static_cast<float>(rand()) / static_cast<float>(RAND_MAX / 30), rand(), WaitMode::DONT_WAIT);
    motion_.fix_pitch();
    motion_.fix_position(MakePoint2(static_cast<double>(rand()) / static_cast<double>(RAND_MAX / 100),
        static_cast<double>(rand()) / static_cast<double>(RAND_MAX / 100)), static_cast<MoveMode>(rand() % 2),
        rand(), WaitMode::DONT_WAIT);
}

int main(int argc, char* argv[])
{
    auto communicator = ipc::init(argc, argv, Mission::NODE_NAME);
    Mission mission(communicator);

    ipc::EventLoop loop(10);
    while (loop.ok()) {
        mission.publish_commands();
    }
    return 0;
}