#pragma once

#include <video/MsgFoundGate.h>
#include <config_reader/yaml_reader.h>

#include <opencv2/opencv.hpp>

#include <algorithm>

#include "common.h"
#include "rec_factory.h"
#include "stripe.h"

class GateRecognizer
{
public:
    GateRecognizer(const YamlReader& cfg) : cfg_(cfg) {}
    video::MsgFoundGate find(const cv::Mat& frame, cv::Mat& out, Mode mode);

private:
    YamlReader cfg_;

    std::vector<Stripe> take_leg(std::vector<Stripe>& legs);
    void draw_gate(cv::Mat& img, const std::vector<Stripe>& red_leg, const std::vector<Stripe>& green_leg);
    video::MsgFoundGate msg(const std::vector<Stripe>& red_leg, const std::vector<Stripe>& green_leg);

    bool is_horizontal(Stripe stripe) {
        return std::abs(stripe.line.first.x - stripe.line.second.x) > std::abs(stripe.line.first.y - stripe.line.second.y);
    }
};
