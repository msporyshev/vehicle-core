#include "channel.h"
#include "fargate.h"
#include "objects.h"

boost::optional<Stripe> ChannelRecognizer::find_stripe(const cv::Mat& frame, cv::Mat& out, int x)
{
    boost::optional<Stripe> result;

    FindContours find_contours(cfg_);
    MinMaxStripes stripes_from_contours(cfg_);
    FilterStripes stripes_filter(cfg_.node("stripes"));

    cv::Mat bin = frame.clone();

    for (int i = 0; i < bin.rows; i++) {
        for (int j = 0; j < bin.cols; j++) {
            if (j < x - border_delta_.get() || j > x + border_delta_.get()) {
                bin.at<int>(i, j) = 0;
            }
        }
    }

    auto contours = find_contours.process(bin);
    auto stripes_raw = stripes_from_contours.process(contours);
    auto stripes = stripes_filter.process(stripes_raw);

    if (stripes.empty()) {
        return result;
    }

    double max_len = 0;
    Stripe max_stripe = stripes.front();
    for (auto& stripe : stripes) {
        if (stripe.len() > max_stripe.len()) {
            max_stripe = stripe;
        }
    }

    max_stripe.draw(out, Color::Yellow, 2);

    return max_stripe;
}

boost::optional<vision::MsgFoundGate> ChannelRecognizer::find(const cv::Mat& frame, cv::Mat& out, Mode mode)
{
    YamlReader cfg("vision.yml", "vision");
    FarGateRecognizer gate_recognizer(cfg.node("fargate"));

    auto msg = gate_recognizer.find(frame, out, mode);

    if (!msg) {
        return msg;
    }


    ImagePipeline processor(mode);
    if (enable_correction_.get() == 1) {
        processor << HistEqualizer(cfg_.node("equalizer"));
    }

    processor << BinarizerHSV(cfg_.node("binarizer"))
        << FrameDrawer(cfg_)
        << MedianFilter(cfg_.node("median_blur")) ;


    auto bin = processor.process(frame);

    auto left_stripe = find_stripe(bin, out, msg->gate.front().left.begin.x);
    auto right_stripe = find_stripe(bin, out, msg->gate.front().right.begin.x);

    if (left_stripe) {
        msg->gate.front().left = left_stripe->to_msg();
    }
    if (right_stripe) {
        msg->gate.front().right = right_stripe->to_msg();
    }

    return msg;
}