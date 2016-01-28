#include <iostream>
#include <functional>

#include <video/MsgVideoFrame.h>
#include <video/MsgFoundBin.h>
#include <video/CmdSwitchCamera.h>

#include <camera/MsgCameraFrame.h>

#include <libipc/ipc.h>
#include <yaml_reader.h>

#include "image_pipeline.h"
#include "image_algorithm.h"
#include "recognizer.h"
#include "rec_factory.h"

using namespace std;
using namespace video;
using namespace camera;

template<typename Msg>
void on_receive(const Msg& msg) {
    ROS_INFO_STREAM("Received " << ipc::classname(msg));
}

void rgb_to_gray(const cv::Mat& frame, cv::Mat& out)
{
    cv::cvtColor(frame, out, CV_BGR2GRAY);
}

int main(int argc, char** argv) {
    auto comm = ipc::init(argc, argv, "video");

    auto found_bucket_pub = comm.advertise<MsgFoundBin>();
    auto frame_pub = comm.advertise<MsgVideoFrame>();

    comm.subscribe_cmd(on_receive<CmdSwitchCamera>);
    comm.subscribe("camera", on_receive<MsgCameraFrame>);

    cv::Mat img = cv::imread("/Users/msporyshev/Downloads/00000002.png");
    cv::imshow("asdfasdf", img);



    ImagePipeline pipe(Mode::Debug);
    pipe << rgb_to_gray
        << bind(cv::cvtColor, _1, _2, CV_BGR2GRAY, 0);

    cv::Mat res = pipe.process(img);
    imshow("gray", res);

    cv::waitKey();

    ipc::EventLoop loop(10);
    while (loop.ok()) {
        MsgVideoFrame img;
        frame_pub.publish(img);

        ROS_INFO_STREAM("Published " << ipc::classname(img));


        MsgFoundBin fb_msg;
        found_bucket_pub.publish(fb_msg);

        ROS_INFO_STREAM("Published " << ipc::classname(fb_msg));
    }
}