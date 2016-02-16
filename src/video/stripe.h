#pragma once

#include <video/MsgFoundBin.h>
#include <config_reader/yaml_reader.h>

#include <opencv2/opencv.hpp>

#include "rec_factory.h"

using Segment = std::pair<cv::Point2d, cv::Point2d>;

struct Stripe {
    Segment line, width;

    Stripe(Segment line = Segment(), Segment width = Segment()): line(line), width(width) {}

    double length() {
        return norm(line.first - line.second);
    }
};

class StripeRecognizer
{
public:
    StripeRecognizer(const YamlReader& cfg): cfg_(cfg) {}

    video::MsgFoundBin find(const cv::Mat& frame, cv::Mat& out, Mode mode);
private:
    YamlReader cfg_;

    video::MsgFoundBin fill_msg(const std::vector<cv::Point>& stripes);
    std::vector<cv::Point> find_stripe(cv::Mat& img);
    std::vector<Stripe> find_stripe_on_bin_img(cv::Mat& img);

    // Для данного массива точек находится минимаксная регрессия, которая "обрезается" в соответствии с размерами контура.
    Stripe min_max_regression_segment(const std::vector<cv::Point> poly, double EPS = 1e-4);

    AUTOPARAM(double, sides_ratio_);
    AUTOPARAM(double, approx_diff_);
    AUTOPARAM_OPTIONAL(double, min_stripe_width_, 0);
    AUTOPARAM_OPTIONAL(double, max_stripe_width_, 0);
    AUTOPARAM_OPTIONAL(double, min_stripe_length_, 0);
    AUTOPARAM_OPTIONAL(double, max_stripe_length_, 0);
    AUTOPARAM_OPTIONAL(double, max_approx_count_, 0);
};

REGISTER_RECOGNIZER(StripeRecognizer, stripe);


