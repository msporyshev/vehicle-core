#pragma once

#include <video/MsgFoundStripe.h>
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

    boost::optional<video::MsgFoundStripe> find(const cv::Mat& frame, cv::Mat& out, Mode mode);
    std::vector<Stripe> find_stripe(const cv::Mat& img);
private:
    YamlReader cfg_;

    video::MsgFoundStripe msg(const std::vector<Stripe>& stripes);
    std::vector<Stripe> find_stripe_on_bin_img(const cv::Mat& img);
    void draw_stripe(cv::Mat& img, const std::vector<Stripe>& stripes);

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

