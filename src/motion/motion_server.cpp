#include "motion_server.h"

#include <cstdlib>
#include <ctime>

#include <libipc/ipc.h>

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

MotionServer::MotionServer() 
{
    srand(time(NULL));
}

MotionServer::~MotionServer() {}

void MotionServer::init_ipc(int argc, char* argv[], const string& node_name)
{
    auto communicator = ipc::init(argc, argv, node_name);

    cmd_status_pub_ = communicator.advertise<motion::MsgCmdStatus>();
    regul_pub_ = communicator.advertise<motion::MsgRegul>();

    communicator.subscribe_cmd(&MotionServer::handle_command<motion::CmdFixDepth>, this);
    communicator.subscribe_cmd(&MotionServer::handle_command<motion::CmdFixDepthConf>, this);
    communicator.subscribe_cmd(&MotionServer::handle_command<motion::CmdFixHeading>, this);
    communicator.subscribe_cmd(&MotionServer::handle_command<motion::CmdFixHeadingConf>, this);
    communicator.subscribe_cmd(&MotionServer::handle_command<motion::CmdFixPitch>, this);
    communicator.subscribe_cmd(&MotionServer::handle_command<motion::CmdFixPitchConf>, this);
    communicator.subscribe_cmd(&MotionServer::handle_command<motion::CmdFixPosition>, this);
    communicator.subscribe_cmd(&MotionServer::handle_command<motion::CmdFixPositionConf>, this);
    communicator.subscribe_cmd(&MotionServer::handle_command<motion::CmdFixThrust>, this);
    communicator.subscribe_cmd(&MotionServer::handle_command<motion::CmdFixVelocity>, this);
    communicator.subscribe_cmd(&MotionServer::handle_command<motion::CmdFixVert>, this);
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
    MotionServer server;

    server.init_ipc(argc, argv, MotionServer::NODE_NAME);
    ipc::EventLoop loop(10);
    while(loop.ok()) {
        server.create_and_publish_regul();
        server.create_and_publish_cmd_status();
    }
    
    return 0;
}