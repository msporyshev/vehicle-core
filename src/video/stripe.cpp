#include "stripe.h"
#include "image_pipeline.h"
#include "image_algorithm.h"
#include "rec_factory.h"

using namespace video;

MsgFoundBin StripeRecognizer::find(const cv::Mat& frame, cv::Mat& out, Mode mode)
{
    ImagePipeline processor(mode);
    processor << BinarizerHSV(cfg_)
        << FrameDrawer(cfg_)
        << MedianBlur(cfg_);

    out = processor.process(frame);

    std::vector<cv::Point> stripes = find_stripe(out);

    size_t n = stripes.size() / 4;
    if (n > 2) n = 2;
    std::vector<cv::Point> tmp;
    for (size_t i = 0; i < n*4; i += 4) {
        for (int j = 0; j < 4; j++)
            tmp.push_back(stripes[i+j]);
    }

    return fill_msg(tmp);
}

MsgFoundBin StripeRecognizer::fill_msg(const std::vector<cv::Point>& stripes)
{
    MsgFoundBin m;
    int stripes_count = stripes.size() / 4;
    for (size_t i = 0; i < stripes_count; i += 4) {
        MsgBin bin;
        for (int j = i; j < i + 4; ++j) {
            bin.rows[j] = stripes[j].y;
            bin.cols[j] = stripes[j].x;
        }
        m.bins.push_back(bin);
    }

    return m;
} 

std::vector<cv::Point> StripeRecognizer::find_stripe(cv::Mat& img)
{
    std::vector<Stripe> raw_stripes, stripes;
    std::vector<cv::Point> result;

    raw_stripes = find_stripe_on_bin_img(img);

    //TODO: Вынести в константы
    cv::Scalar orange_color(30, 75, 250);

    cfg_.read_param(sides_ratio_, "sides_ratio");

    for (auto stripe : raw_stripes) {
        double side_ratio = stripe.length() / norm(stripe.width.first - stripe.width.second);
        if (side_ratio > sides_ratio_) {
            stripes.push_back(stripe);
            line(img, stripe.line.first, stripe.line.second, orange_color, 2, CV_AA, 0);
        }
    }

    for (const auto& stripe : stripes) {
        auto dir = (stripe.width.first - stripe.width.second) * 0.5;

        std::vector<cv::Point> tmp = {stripe.line.first - dir,
            stripe.line.first + dir,
            stripe.line.second + dir,
            stripe.line.second - dir};

        cv::Point center;
        for (auto point : tmp) {
            center += point;
        }

        center.x /= 4;
        center.y /= 4;

        result.insert(result.end(), tmp.begin(), tmp.end());
        circle(img, center, 3, cv::Scalar(0, 255, 0), -1);
        for (size_t i = 0; i < 4; i++) {
            line(img, tmp[i], tmp[(i+1) % 4], orange_color, 1, CV_AA, 0);
        }
    }

    return result;
}

std::vector<Stripe> StripeRecognizer::find_stripe_on_bin_img(cv::Mat& img)
{
    cv::Scalar color(30, 75,  250);

    double MIN_WIDTH, MAX_WIDTH, MIN_LENGTH, MAX_LENGTH, APPROX_DIFF;
    int MAX_APPROX_COUNT;

    bool use_min_width = cfg_.is_param_readable(MIN_WIDTH, "min_stripe_width");
    bool use_max_width = cfg_.is_param_readable(MAX_WIDTH, "max_stripe_width");

    bool use_min_length = cfg_.is_param_readable(MIN_LENGTH, "min_stripe_length");
    bool use_max_length = cfg_.is_param_readable(MAX_LENGTH, "max_stripe_length");

    cfg_.read_param(APPROX_DIFF, "approx_diff");
    bool use_max_approx_count = cfg_.is_param_readable(MAX_APPROX_COUNT, "max_approx_count");

    std::vector<std::vector<cv::Point> > contours, approxes;
    std::vector<cv::Vec4i> hierarchy;

    findContours(img, contours, hierarchy,
        CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

    for (size_t i = 0; i < contours.size(); i++) {
        std::vector<cv::Point> approx;
        approxPolyDP(contours[i], approx, APPROX_DIFF, false);
        if (use_max_approx_count && approx.size() > MAX_APPROX_COUNT) {
            continue;
        }

        approxes.push_back(approx);
    }

    std::vector<Stripe> stripes;
    for (int i = 0; i < approxes.size(); i++) {
        auto stripe = min_max_regression_segment(approxes[i]);

        stripes.push_back(stripe);
    }

    std::vector<Stripe> res;

    for (const auto& stripe : stripes) {
        double length = norm(stripe.line.first - stripe.line.second);
        double width = norm(stripe.width.first - stripe.width.second);

        if (use_max_length && length > MAX_LENGTH) {
            continue;
        }

        if (use_min_length && length < MIN_LENGTH) {
            continue;
        }

        if (use_max_width && width > MAX_WIDTH) {
            continue;
        }

        if (use_min_width && width < MIN_WIDTH) {
            continue;
        }

        res.push_back(stripe);
    }

    return res;
}





