#include <vision/MsgFoundBin.h>
#include <config_reader/yaml_reader.h>
#include <point/point.h>

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <vector>

#include "common.h"
#include "rec_factory.h"
#include "image_pipeline.h"
#include "image_algorithm.h"

#include <vision/MsgBin.h>

using namespace std;

namespace {

FuncWrapper<cv::Mat, std::vector<Stripe>> stripe_pipeline(const YamlReader& cfg) {
    auto res = HistEqualizer(cfg.node("equalizer"))
        + BinarizerHSV(cfg.node("binarizer"))
        + FrameDrawer(cfg)
        + MedianFilter(cfg.node("median_blur"))
        + FindContours(cfg)
        + MinMaxStripes(cfg)
        + FilterStripes(cfg.node("stripes"))
        ;

    return res;
}

} // namespace

class BinRecognizer
{
public:
    BinRecognizer(const YamlReader& cfg): cfg_(cfg)
    {
        orange_pipe_ = stripe_pipeline(cfg_.node("orange"));
        white_pipe_ = stripe_pipeline(cfg_.node("white"));
    }

    boost::optional<vision::MsgFoundBin> find(const cv::Mat& frame, cv::Mat& out, Mode mode)
    {
        boost::optional<vision::MsgFoundBin> result;

        auto ostripes = orange_pipe_.process(frame, out, mode);
        auto wstripes = white_pipe_.process(frame, out, mode);

        vision::MsgFoundBin msg;
        for (auto& stripe : wstripes) {
            vision::MsgBin msg_bin;
            msg_bin.stripe = stripe.to_msg();
            msg_bin.locked = false;

            stripe.draw(out, Color::Yellow, 2);

            msg.bins.push_back(msg_bin);
        }

        for (auto& stripe : ostripes) {
            vision::MsgBin msg_bin;
            msg_bin.stripe = stripe.to_msg();
            msg_bin.locked = true;
            msg.bins.push_back(msg_bin);

            stripe.draw(out, Color::Orange, 2);
        }

        return msg;
    }

private:
    YamlReader cfg_;

    FuncWrapper<cv::Mat, std::vector<Stripe>> orange_pipe_;
    FuncWrapper<cv::Mat, std::vector<Stripe>> white_pipe_;
};

REGISTER_RECOGNIZER(BinRecognizer, bin);
