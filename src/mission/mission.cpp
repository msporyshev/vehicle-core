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

#include <video/MsgFoundBin.h>

using namespace std;

const string Mission::NODE_NAME = "mission";

Mission::Mission(ipc::Communicator& communicator) :
    communicator_(communicator),
    motion_(RobosubMotionClient(communicator))
{
    srand(time(NULL));
    this->init_ipc();
}

Mission::~Mission()
{}

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

    ipc::EventLoop loop(50);
    while (loop.ok()) {
        // mission.publish_commands();
    }
    return 0;
}