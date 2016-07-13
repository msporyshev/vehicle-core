#include "mission.h"

#include <cstdlib>
#include <ctime>

using namespace std;

int main(int argc, char* argv[])
{
    auto communicator = ipc::init(argc, argv, "mission");
    Mission mission(communicator);
    mission.push_default_tasks();

    ipc::EventLoop loop(10);
    while(loop.ok()) {
        mission.run();
    }

    return 0;
}
