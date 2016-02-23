#include <video/MsgFoundGate.h>
#include <config_reader/yaml_reader.h>
#include <point/point.h>

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <vector>

#include "common.h"
#include "rec_factory.h"
#include "image_pipeline.h"
#include "image_algorithm.h"

using namespace std;

class FarGateRecognizer
{
public:
    FarGateRecognizer(const YamlReader& cfg) : cfg_(cfg) {}
    boost::optional<video::MsgFoundGate> find(const cv::Mat& frame, cv::Mat& out, Mode mode)
    {
        boost::optional<video::MsgFoundGate> result;

        ImagePipeline pipe(mode);
        pipe
            << MedianFilter(cfg_.node("median_big"))
            << AbsDiffFilter(cfg_, frame)
            << GrayScale()
            << SobelFilter(cfg_)
            ;
        auto preprocessed = pipe.process(frame);

        vector<int> count(256);
        for (int i = 0; i < preprocessed.rows; i++) {
            for (int j = 0; j < preprocessed.cols; j++) {
                int val = preprocessed.at<uchar>(i, j);
                count[val]++;
            }
        }

        int most_freq = 0;
        int thresh = 0;
        for (int i = count.size() - 1; i >= 0; i--) {
            most_freq += count[i];
            double ratio = most_freq * 1.0 / 400 / 300;
            if (ratio > grad_ratio_.get()) {
                ROS_INFO_STREAM("ratio: " << ratio);
                thresh = i;
                ROS_INFO_STREAM("thresh: " << thresh);
                break;
            }
        }

        cv::Mat threshed;
        cv::threshold(preprocessed, threshed, thresh, 255, cv::THRESH_BINARY);
        MedianFilter median(cfg_.node("median"));
        auto bin = median.process(threshed);
        if (mode == Mode::Debug) {
            cv::imshow("most common", threshed);
        }

        int cell_pixels = cell_pixels_.get();
        vector<int> xcount(bin.cols / cell_pixels + 1);
        int all_count = 0;
        for (int i = 0; i < bin.cols; i++) {
            for (int j = 0; j < bin.rows; j++) {
                if (bin.at<uchar>(j, i) > 0) {
                    xcount[i / cell_pixels]++;
                    all_count++;
                }
            }
        }

        vector<int> smax(xcount.size());
        vector<int> smaxi(xcount.size());
        smax.back() = xcount.back();
        for (int i = (int)xcount.size() - 2; i >= 0; i--) {
            if (xcount[i] > smax[i + 1]) {
                smax[i] = xcount[i];
                smaxi[i] = i;
            } else {
                smax[i] = smax[i + 1];
                smaxi[i] = smaxi[i + 1];
            }
        }

        int bestmin = 0;
        int first_peak;
        int second_peak;
        int first_peak_x;
        int second_peak_x;
        int r = min_gate_width_.get() / cell_pixels;
        for (int i = 0; i < xcount.size() - r; i++) {
            int j = i + r;

            int peak_min = min(xcount[i], smax[j]);
            if (peak_min > bestmin) {
                first_peak_x = i * cell_pixels;
                first_peak = xcount[i];
                second_peak_x = smaxi[j] * cell_pixels;
                second_peak = smax[j];
                bestmin = peak_min;
            }
        }

        pair<double, double> proba(1.0 * first_peak / cell_pixels / bin.rows,
            1.0 * second_peak / cell_pixels / bin.rows);

        ROS_INFO_STREAM("Peak proba: " << proba.first << " " << proba.second);

        Stripe left(Segment(cv::Point2d(first_peak_x, 0), cv::Point2d(first_peak_x, 400)));
        Stripe right(Segment(cv::Point2d(second_peak_x, 0), cv::Point2d(second_peak_x, 400)));

        left.draw(out, Color::Orange, 2);
        right.draw(out, Color::Orange, 2);

        video::MsgFoundGate m;
        m.gate.left = left.to_msg();
        m.gate.right = right.to_msg();

        return m;
    }

private:
    YamlReader cfg_;

    AUTOPARAM(double, grad_ratio_);
    AUTOPARAM(int, min_gate_width_);
    AUTOPARAM_OPTIONAL(int, cell_pixels_, 1);
};

REGISTER_RECOGNIZER(FarGateRecognizer, fargate);
