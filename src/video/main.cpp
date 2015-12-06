#include <iostream>

#include <video/MsgVideoFrame.h>
#include <video/MsgFoundBin.h>
#include <video/CmdSwitchCamera.h>

#include <camera/MsgCameraFrame.h>

#include <libipc/ipc.h>
#include <yaml_reader.h>

using namespace std;
using namespace video;
using namespace camera;

template<typename Msg>
void on_receive(const Msg& msg) {
    cout << msg << endl;
}

int main(int argc, char** argv) {
    auto comm = ipc::init(argc, argv, "video");

    auto found_bucket_pub = comm.advertise<MsgFoundBin>();
    auto frame_pub = comm.advertise<MsgVideoFrame>();

    comm.subscribe_cmd(on_receive<CmdSwitchCamera>);
    comm.subscribe("camera", on_receive<MsgCameraFrame>);


    ipc::EventLoop loop(10);
    while (loop.ok()) {
        MsgVideoFrame img;
        frame_pub.publish(img);

        MsgFoundBin fb_msg;
        found_bucket_pub.publish(fb_msg);
    }
}