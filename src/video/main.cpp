#include <iostream>

#include <libipc/ipc.h>

using namespace std;

int main(int argc, char** argv) {
    auto comm = ipc::init(argc, argv, "video");
    cout << "Hello from Video module" << endl;
}