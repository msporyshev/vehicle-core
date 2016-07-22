#pragma once

#include <vision/MsgFoundStripe.h>
#include <config_reader/yaml_reader.h>

#include <opencv2/opencv.hpp>

#include "rec_factory.h"
#include "image_algorithm.h"

class StripeRecognizer
{
public:
    StripeRecognizer(const YamlReader& cfg);

    boost::optional<vision::MsgFoundStripe> find(const cv::Mat& frame, cv::Mat& out, Mode mode);
private:
    YamlReader cfg_;

    FuncWrapper<cv::Mat, std::vector<Stripe>> pipe_;

    AUTOPARAM(bool, enable_col_cor_);
};

REGISTER_RECOGNIZER(StripeRecognizer, stripe);

