#include "stripe.h"
#include "image_pipeline.h"
#include "image_algorithm.h"
#include "rec_factory.h"

#include <algorithm>

#include <point/point.h>

using namespace vision;

boost::optional<MsgFoundStripe> StripeRecognizer::find(const cv::Mat& frame, cv::Mat& out, Mode mode)
{
    boost::optional<MsgFoundStripe> result;

    ImagePipeline processor(mode);
    if (enable_col_cor_.get() == 1) {
        processor << HistEqualizer(cfg_.node("equalizer"));
    }
    processor << BinarizerHSV(cfg_.node("binarizer"))
        << FrameDrawer(cfg_)
        << MedianFilter(cfg_.node("median_blur")) ;

    FindContours find_contours(cfg_);
    MinMaxStripes stripes_from_contours(cfg_);
    FilterStripes stripes_filter(cfg_.node("stripes"));
    auto pre = processor.process(frame);
    auto contours = find_contours.process(pre);
    auto stripes_raw = stripes_from_contours.process(contours);
    auto stripes = stripes_filter.process(stripes_raw);

    if (stripes.empty()) {
        return result;
    }

    MsgFoundStripe msg;
    double max_len = 0;
    Stripe max_stripe = stripes.front();
    for (auto& stripe : stripes) {
        if (stripe.len() > max_stripe.len()) {
            max_stripe = stripe;
        }
    }

    max_stripe.draw(out, Color::Yellow, 2);

    msg.stripes.push_back(max_stripe.to_msg());

    return msg;
}
