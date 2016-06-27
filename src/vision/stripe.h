#pragma once

#include <vision/MsgFoundStripe.h>
#include <config_reader/yaml_reader.h>

#include <opencv2/opencv.hpp>

#include "rec_factory.h"
#include "image_algorithm.h"

class StripeRecognizer
{
public:
    StripeRecognizer(const YamlReader& cfg): cfg_(cfg) {}

    boost::optional<vision::MsgFoundStripe> find(const cv::Mat& frame, cv::Mat& out, Mode mode);
    std::vector<Stripe> find_stripe(const cv::Mat& img);
private:
    YamlReader cfg_;

    vision::MsgFoundStripe msg(const std::vector<Stripe>& stripes);
    std::vector<Stripe> find_stripe_on_bin_img(const cv::Mat& img);
    void draw_stripe(cv::Mat& img, const std::vector<Stripe>& stripes);

    // Для данного массива точек находится минимаксная регрессия, которая "обрезается" в соответствии с размерами контура.
    Stripe min_max_regression_segment(const std::vector<cv::Point> poly, double EPS = 1e-4);

    AUTOPARAM_OPTIONAL(int, enable_col_cor_, 0);
};

REGISTER_RECOGNIZER(StripeRecognizer, stripe);

