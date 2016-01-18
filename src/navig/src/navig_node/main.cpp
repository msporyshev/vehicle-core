#include "navig.h"

int main(int argc, char* argv[])
{
    auto communicator = ipc::init(argc, argv, navig::NODE_NAME);
    navig::init_ipc(communicator);

    ipc::EventLoop loop(10);
    while(loop.ok()) {
        navig::create_and_publish_depth();
        navig::create_and_publish_height();
        navig::create_and_publish_velocity();
    }

    return 0;
}