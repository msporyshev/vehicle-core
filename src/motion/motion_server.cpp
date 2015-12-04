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

    communicator_.subscribe_cmd(&MotionServer::handle_command<motion::CmdFixDepth>, this);
    communicator_.subscribe_cmd(&MotionServer::handle_command<motion::CmdFixDepthConf>, this);
    communicator_.subscribe_cmd(&MotionServer::handle_command<motion::CmdFixHeading>, this);
    communicator_.subscribe_cmd(&MotionServer::handle_command<motion::CmdFixHeadingConf>, this);
    communicator_.subscribe_cmd(&MotionServer::handle_command<motion::CmdFixPitch>, this);
    communicator_.subscribe_cmd(&MotionServer::handle_command<motion::CmdFixPitchConf>, this);
    communicator_.subscribe_cmd(&MotionServer::handle_command<motion::CmdFixPosition>, this);
    communicator_.subscribe_cmd(&MotionServer::handle_command<motion::CmdFixPositionConf>, this);
    communicator_.subscribe_cmd(&MotionServer::handle_command<motion::CmdFixThrust>, this);
    communicator_.subscribe_cmd(&MotionServer::handle_command<motion::CmdFixVelocity>, this);
    communicator_.subscribe_cmd(&MotionServer::handle_command<motion::CmdFixVert>, this);
}

void MotionServer::create_and_publish_cmd_status()
{
    motion::MsgCmdStatus msg;
    msg.status = rand() % 4;
    msg.id = rand();
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