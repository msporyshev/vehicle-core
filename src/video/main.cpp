#include <iostream>

#include <libipc/ipc.h>
#include <yaml_reader.h>

using namespace std;

int main(int argc, char** argv) {
    auto comm = ipc::init(argc, argv, "video");
    cout << "Hello from Video module" << endl;
}