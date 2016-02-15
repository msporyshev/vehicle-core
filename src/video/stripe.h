#pragma once

#include <video/MsgFoundBin.h>
#include <config_reader/yaml_reader.h>

#include <opencv2/opencv.hpp>

#include "common.h"
#include "rec_factory.h"

class StripeRecognizer
{
public:
    StripeRecognizer(const YamlReader& cfg): cfg_(cfg) {}

    video::MsgFoundBin find(const cv::Mat& frame, cv::Mat& out, Mode mode);
private:
    YamlReader cfg_;
    double sides_ratio_;

    video::MsgFoundBin fill_msg(const std::vector<cv::Point>& stripes);
    std::vector<cv::Point> find_stripe(cv::Mat& img);
    std::vector<Stripe> find_stripe_on_bin_img(cv::Mat& img);
};

REGISTER_RECOGNIZER(StripeRecognizer, stripe);


