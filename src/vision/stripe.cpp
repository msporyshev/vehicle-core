#include "stripe.h"
#include "image_pipeline.h"
#include "image_algorithm.h"
#include "rec_factory.h"

#include <algorithm>

#include <point/point.h>

using namespace vision;

StripeRecognizer::StripeRecognizer(const YamlReader& cfg): cfg_(cfg)
{
    pipe_ = HistEqualizer(cfg_.node("equalizer"))
        + BinarizerHSV(cfg_.node("binarizer"))
        + FrameDrawer(cfg_)
        + MedianFilter(cfg_.node("median_blur"))
        + FindContours(cfg_)
        + MinMaxStripes(cfg_)
        + FilterStripes(cfg_.node("stripes"))
        ;
}


boost::optional<MsgFoundStripe> StripeRecognizer::find(const cv::Mat& frame, cv::Mat& out, Mode mode)
{
    boost::optional<MsgFoundStripe> result;

    // auto stripes = pipe_.process(frame, out, mode);

    // if (stripes.empty()) {
    //     return result;
    // }

    // MsgFoundStripe msg;
    // double max_len = 0;
    // Stripe max_stripe = stripes.front();
    // for (auto& stripe : stripes) {
    //     if (stripe.len() > max_stripe.len()) {
    //         max_stripe = stripe;
    //     }
    // }

    // max_stripe.draw(out, Color::Yellow, 2);

    // msg.stripes.push_back(max_stripe.to_msg());

    auto pipe = Grayscale()
        + MedianFilter(cfg_.node("median"));

    auto gray = pipe.process(frame, out, mode);

    for (int i = 0; i < gray.rows; i++) {
        for (int j = 0; j < gray.cols; j++) {

        }
    }

    return msg;
}
