#include "ipc.h"
#include <signal.h>
#include <execinfo.h>
#include <stdlib.h>
#include <unistd.h>


using namespace std;

namespace ipc {

const int Communicator::MSG_QUEUE_SIZE = 1;
const int Communicator::CMD_QUEUE_SIZE = 5;

static void sig_handler(int sig) {
  void *array[10];
  size_t size;

  // get void*'s for all entries on the stack
  size = backtrace(array, 10);

  // print out all the frames to stderr
  fprintf(stderr, "Error: signal %d:\n", sig);
  backtrace_symbols_fd(array, size, STDERR_FILENO);
  exit(1);
}

Communicator init(int argc, char** argv, string node_name) {
    // for (int i = 1; i < NSIG; i++) {
    signal(SIGSEGV, sig_handler);
    // }

    ros::init(argc, argv, node_name);
    return Communicator(node_name);
}

} // namespace ipc
