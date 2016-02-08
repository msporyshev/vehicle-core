#pragma once

#include <ros/ros.h>
#include <opencv2/opencv.hpp>

#include <functional>
#include <type_traits>

#include <config_reader/yaml_reader.h>
#include <libipc/ipc.h>

#include "image_pipeline.h"
#include "common.h"

class RecognizerBase
{
public:
    virtual void process(const cv::Mat& frame, cv::Mat& debug_out, Mode mode, int frameno, Camera camera_type) = 0;

    virtual void init(const YamlReader& cfg,
            Ipc mode,
            ipc::Communicator& comm) = 0;

protected:
    YamlReader cfg_;
    ros::Publisher pub_;
};

#include <video/MsgFoundBin.h>
struct tmp {
    video::MsgFoundBin print (const cv::Mat& frame, cv::Mat& out, Mode mode) {
        return video::MsgFoundBin();
    }
};

template<typename CustomRecognizer>
class Recognizer: public RecognizerBase
{
public:
    using Msg =
        typename std::result_of<decltype(&CustomRecognizer::find)
        (CustomRecognizer, const cv::Mat&, cv::Mat&, Mode)>::type;

    void init(const YamlReader& cfg,
            Ipc mode,
            ipc::Communicator& comm) override
    {
        pub_ = comm.advertise<Msg>();
        ipc_mode_ = mode;
        recognizer_ = std::make_shared<CustomRecognizer>(cfg);
    }

    void process(const cv::Mat& frame, cv::Mat& debug_out, Mode mode, int frameno, Camera camera_type) override
    {
        auto msg = recognizer_->find(frame, debug_out, mode);

        if (ipc_mode_ == Ipc::On) {
            msg.frame_number = frameno;
            msg.camera_type = static_cast<int>(camera_type);

            pub_.publish(msg);
        }
    }

private:
    std::shared_ptr<CustomRecognizer> recognizer_;
    Ipc ipc_mode_ = Ipc::Off;
};