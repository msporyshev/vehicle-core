#include "mission.h"

#include <cstdlib>
#include <ctime>
#include <signal.h>

using namespace std;

int main(int argc, char* argv[])
{
    auto communicator = ipc::init(argc, argv, Mission::NODE_NAME);

    for (int i = 1; i < NSIG; i++) {
        signal(i, sig_handler);
    }

    Mission mission(communicator);
    mission.push_default_tasks();

    ipc::EventLoop loop(10);
    while(loop.ok()) {
        mission.run();
    }

    return 0;
}
