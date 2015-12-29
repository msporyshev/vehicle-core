/**
\file
\brief Реализация модуля регуляторов

В данном файле находятся реализации методов, объявленных в motion_server.h
Также здесь находится main
*/

#include "motion_server.h"

#include <cstdlib>
#include <ctime>

#include <motion/MsgCmdStatus.h>
#include <motion/MsgRegul.h>

#include <motion/CmdFixDepth.h>
#include <motion/CmdFixDepthConf.h>
#include <motion/CmdFixHeading.h>
#include <motion/CmdFixHeadingConf.h>
#include <motion/CmdFixPitch.h>
#include <motion/CmdFixPitchConf.h>
#include <motion/CmdFixPosition.h>
#include <motion/CmdFixPositionConf.h>
#include <motion/CmdFixThrust.h>
#include <motion/CmdFixVelocity.h>
#include <motion/CmdFixVert.h>

#include <navig/MsgNavigAngles.h>
#include <navig/MsgNavigDepth.h>
#include <navig/MsgNavigHeight.h>
#include <navig/MsgNavigPosition.h>

using namespace std;

const string MotionServer::NODE_NAME = "motion";

MotionServer::MotionServer(ipc::Communicator& communicator) :
    communicator_(communicator)
{
    srand(time(NULL));
    this->init_ipc();
}

MotionServer::~MotionServer() {}

void MotionServer::init_ipc()
{
    cmd_status_pub_ = communicator_.advertise<motion::MsgCmdStatus>();
    regul_pub_ = communicator_.advertise<motion::MsgRegul>();

    /**
        Подписка на все команды от MotionClient (они приходят либо из миссии, либо из пульта)
    */
    communicator_.subscribe_cmd(&MotionServer::handle_message<motion::CmdFixDepth>, this);
    communicator_.subscribe_cmd(&MotionServer::handle_message<motion::CmdFixDepthConf>, this);
    communicator_.subscribe_cmd(&MotionServer::handle_message<motion::CmdFixHeading>, this);
    communicator_.subscribe_cmd(&MotionServer::handle_message<motion::CmdFixHeadingConf>, this);
    communicator_.subscribe_cmd(&MotionServer::handle_message<motion::CmdFixPitch>, this);
    communicator_.subscribe_cmd(&MotionServer::handle_message<motion::CmdFixPitchConf>, this);
    communicator_.subscribe_cmd(&MotionServer::handle_message<motion::CmdFixPosition>, this);
    communicator_.subscribe_cmd(&MotionServer::handle_message<motion::CmdFixPositionConf>, this);
    communicator_.subscribe_cmd(&MotionServer::handle_message<motion::CmdFixThrust>, this);
    communicator_.subscribe_cmd(&MotionServer::handle_message<motion::CmdFixVelocity>, this);
    communicator_.subscribe_cmd(&MotionServer::handle_message<motion::CmdFixVert>, this);

    /**
        Подписка на сообщения от навига
    */
    communicator_.subscribe("navig", &MotionServer::handle_message<navig::MsgNavigAngles>, this);
    communicator_.subscribe("navig", &MotionServer::handle_message<navig::MsgNavigDepth>, this);
    communicator_.subscribe("navig", &MotionServer::handle_message<navig::MsgNavigHeight>, this);
    communicator_.subscribe("navig", &MotionServer::handle_message<navig::MsgNavigPosition>, this);
}

void MotionServer::create_and_publish_cmd_status()
{
    motion::MsgCmdStatus msg;
    msg.status = rand() % 4;
    msg.id = rand();
    ROS_INFO_STREAM("Published " << ipc::classname(msg));
    cmd_status_pub_.publish(msg);
}

void MotionServer::create_and_publish_regul()
{
    motion::MsgRegul msg;
    msg.tx = static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / 2)) - 1;
    msg.ty = static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / 2)) - 1;
    msg.tz = static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / 2)) - 1;
    msg.mx = static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / 2)) - 1;
    msg.my = static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / 2)) - 1;
    msg.mz = static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / 2)) - 1;
    ROS_INFO_STREAM("Published " << ipc::classname(msg));
    regul_pub_.publish(msg);
}

int main(int argc, char* argv[])
{
    auto communicator = ipc::init(argc, argv, MotionServer::NODE_NAME);
    MotionServer server(communicator);

    ipc::EventLoop loop(10);
    while(loop.ok()) {
        server.create_and_publish_regul();
        server.create_and_publish_cmd_status();
    }
    
    return 0;
}