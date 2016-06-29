#pragma once

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <boost/optional.hpp>

#include <navig/MsgOdometry.h>

#include <functional>
#include <type_traits>
#include <memory>

#include <config_reader/yaml_reader.h>
#include <libipc/ipc.h>

#include "image_pipeline.h"
#include "common.h"


class RecognizerBase
{
public:
    virtual void process(const cv::Mat& frame, cv::Mat& debug_out,
            Mode mode, int frameno, Camera camera_type, navig::MsgOdometry odometry) = 0;

    virtual void init(const YamlReader& cfg,
            ipc::CommunicatorPtr comm) = 0;

protected:
    YamlReader cfg_;
    ros::Publisher pub_;
};

template<typename RecognizerImpl>
class Recognizer: public RecognizerBase
{
public:
    using MsgOptional =
        typename std::result_of<decltype(&RecognizerImpl::find)
        (RecognizerImpl, const cv::Mat&, cv::Mat&, Mode)>::type;

    using Msg = typename MsgOptional::value_type;

    void init(const YamlReader& cfg,
            ipc::CommunicatorPtr comm) override
    {
        ipc_mode_ = comm ? Ipc::On : Ipc::Off;
        if (comm) {
            pub_ = comm->advertise<Msg>();
        }

        recognizer_ = std::make_shared<RecognizerImpl>(cfg);
    }

    void process(const cv::Mat& frame, cv::Mat& debug_out,
            Mode mode, int frameno, Camera camera_type, navig::MsgOdometry odometry) override
    {
        auto msg = recognizer_->find(frame, debug_out, mode);

        if (msg && ipc_mode_ == Ipc::On) {
            msg->header.stamp = ros::Time::now();
            msg->frame_number = frameno;
            msg->camera_type = static_cast<int>(camera_type);
            msg->odometry = odometry;

            pub_.publish(*msg);
        }
    }

private:
    std::shared_ptr<RecognizerImpl> recognizer_;
    Ipc ipc_mode_ = Ipc::Off;
};
