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
using namespace cv;

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

vector<cv::Point2d> points(Stripe stripe)
{
    vector<cv::Point2d> result;

    cv::Point2d w = 0.5 * (stripe.w.first - stripe.w.second);

    result.push_back(stripe.l.first + w);
    result.push_back(stripe.l.first - w);
    result.push_back(stripe.l.second - w);
    result.push_back(stripe.l.second + w);

    return result;
}

vector<cv::Point2d> scale(const vector<cv::Point2d>& p, double ratio)
{
    vector<cv::Point2d> q;
    int min_x, min_y, max_x, max_y, center_x, center_y;

    min_x = max_x = p[0].x;
    min_y = max_y = p[0].y;
    for(size_t i = 0; i < p.size(); i++) {
        if(p[i].x < min_x) min_x = p[i].x;
        if(p[i].x > max_x) max_x = p[i].x;
        if(p[i].y < min_y) min_y = p[i].y;
        if(p[i].y > max_y) max_y = p[i].y;
    }
    center_x = (min_x + max_x) / 2;
    center_y = (min_y + max_y) / 2;

    for(size_t i = 0; i < p.size(); i++) {
        q.push_back(cv::Point2d(((p[i].x - center_x) * ratio) + center_x,
            ((p[i].y - center_y) * ratio) + center_y));
    }

    return q;
}

bool color_compare(Mat &frame, vector<cv::Point2d>& p, vector<cv::Point2d>& q, int color_difference)
{
    double p_color = 0, q_color = 0;
    int p_count = 0, q_count = 0;
    for(size_t i = 0; i < p.size(); i++) {
        double p_step, q_step = 0;
        if(abs(p[(i+1)%4].x - p[i%4].x) > abs(p[(i+1)%4].y - p[i%4].y)) {
            p_step = fabs(1. / (p[(i+1)%4].x - p[i%4].x));
        }
        else {
            p_step = fabs(1. / (p[(i+1)%4].y - p[i%4].y));
        }
        if(abs(q[(i+1)%4].x - q[i%4].x) > abs(q[(i+1)%4].y - q[i%4].y)) {
            q_step = fabs(1. / (q[(i+1)%4].x - q[i%4].x));
        }
        else {
            q_step = fabs(1. / (q[(i+1)%4].y - q[i%4].y));
        }
        for(double t = 0; t < 1; t += p_step) {
            int X = floor((p[(i+1)%4].x - p[i%4].x)*t + p[i%4].x);
            int Y = floor((p[(i+1)%4].y - p[i%4].y)*t + p[i%4].y);
            if(X > 0 && X < frame.cols && Y > 0 && Y < frame.rows) {
                p_color += (int)frame.at<uchar>(Y, X);
                p_count++;
            }
        }
        for(double t = 0; t < 1; t += q_step) {
            int X = floor((q[(i+1)%4].x - q[i%4].x)*t + q[i%4].x);
            int Y = floor((q[(i+1)%4].y - q[i%4].y)*t + q[i%4].y);
            if(X > 0 && X < frame.cols && Y > 0 && Y < frame.rows) {
                q_color += (int)frame.at<uchar>(Y, X);
                q_count++;
            }
        }

    }

    if (p_count == 0 || q_count == 0) {
        return false;
    }

    p_color /= p_count;
    q_color /= q_count;
    cout << "p_color: " << p_color << endl;
    cout << "q_color: " << q_color << endl;
    ROS_INFO_STREAM("color difference: " << q_color - p_color);
    ROS_INFO_STREAM("destination color difference: " << color_difference);
    if(q_color - p_color > color_difference){
        return true;
    }
    else {
        return false;
    }
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

        // auto ostripes = orange_pipe_.process(frame, out, mode);
        auto wstripes = white_pipe_.process(frame, out, mode);

        auto cfg = cfg_.node("white");
        auto bin_pipe = HistEqualizer(cfg.node("equalizer"))
            + BinarizerHSV(cfg.node("binarizer"))
            + FrameDrawer(cfg)
            + MedianFilter(cfg.node("median_blur"));

        auto bin = bin_pipe.process(frame, out, mode);

        vision::MsgFoundBin msg;

        Mat gray;
        cvtColor(frame, gray, CV_BGR2GRAY);

        for (auto& stripe : wstripes) {
            auto scale_high = scale(points(stripe), scale_high_.get());
            auto scale_low = scale(points(stripe), scale_low_.get());

            if (!color_compare(bin, scale_low, scale_high, color_diff_.get())) {
                continue;
            }

            vision::MsgBin msg_bin;
            msg_bin.stripe = stripe.to_msg();
            msg_bin.locked = false;

            stripe.draw(out, Color::Yellow, 2);

            msg.bins.push_back(msg_bin);
        }

        // for (auto& stripe : ostripes) {
        //     vision::MsgBin msg_bin;
        //     msg_bin.stripe = stripe.to_msg();
        //     msg_bin.locked = true;
        //     msg.bins.push_back(msg_bin);

        //     stripe.draw(out, Color::Orange, 2);
        // }


        return msg;
    }

private:
    YamlReader cfg_;

    FuncWrapper<cv::Mat, std::vector<Stripe>> orange_pipe_;
    FuncWrapper<cv::Mat, std::vector<Stripe>> white_pipe_;

    AUTOPARAM(double, scale_low_);
    AUTOPARAM(double, scale_high_);
    AUTOPARAM(double, color_diff_);
};

REGISTER_RECOGNIZER(BinRecognizer, bin);
