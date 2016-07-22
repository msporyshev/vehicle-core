#include <vision/MsgFoundGate.h>
#include <config_reader/yaml_reader.h>
#include <point/point.h>

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <vector>

#include "fargate.h"
#include "common.h"
#include "rec_factory.h"
#include "image_pipeline.h"
#include "image_algorithm.h"

using namespace std;

boost::optional<vision::MsgFoundGate> FarGateRecognizer::find(const cv::Mat& frame, cv::Mat& out, Mode mode)
{
    boost::optional<vision::MsgFoundGate> result;

    ImagePipeline white_filter(mode);
    white_filter
        << BinarizerHSV(cfg_.node("hsv"), false)
        << DilateSquare(cfg_.node("dilate"))
        << Invert()
        ;
    auto mask = white_filter.process(frame);

    MedianFilter median(cfg_.node("median"));
    auto filtered = HistEqualizer(cfg_).process(median.process(frame));

    ImagePipeline pipe(mode);
    pipe
        << HistEqualizer(cfg_)
        << MedianFilter(cfg_.node("median_big"))
        << AbsDiffFilter(cfg_, filtered)
        << SobelFilter(cfg_)
        << GrayScale()
        // << ApplyMask(mask)
        << MostCommonFilter(cfg_)
        ;
    auto bin = pipe.process(frame);

    int cell_pixels = cell_pixels_.get();
    vector<int> xcount((bin.cols + cell_pixels - 1)/ cell_pixels);
    int all_count = 0;
    for (int i = 0; i < bin.cols; i++) {
        for (int j = 0; j < bin.rows; j++) {
            if (bin.at<uchar>(j, i) > 0) {
                xcount[i / cell_pixels]++;
                all_count++;
            }
        }
    }

    if (mode == Mode::Debug) {
        cv::Mat hist_frame = frame;
        Hist hist(xcount);
        hist.draw(hist_frame, Color::Orange);
        cv::imshow("histogram", hist_frame);
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
    int first_peak = 0;
    int second_peak = 0;
    int first_peak_x = -1;
    int second_peak_x = -1;
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

    vision::MsgFoundGate m;

    if (first_peak_x == -1) {
        return m;
    }

    pair<double, double> proba(1.0 * first_peak / cell_pixels / bin.rows,
        1.0 * second_peak / cell_pixels / bin.rows);

    ROS_INFO_STREAM("Peak proba: " << proba.first << " " << proba.second);

    Stripe left(Segment(cv::Point2d(first_peak_x, 0), cv::Point2d(first_peak_x, 400)));
    Stripe right(Segment(cv::Point2d(second_peak_x, 0), cv::Point2d(second_peak_x, 400)));

    Color color = proba.first > hough_thresh_.get() ? Color::Orange : Color::Green;
    left.draw(out, color, 2);

    color = proba.second > hough_thresh_.get() ? Color::Orange : Color::Green;
    right.draw(out, color, 2);


    if (proba.first < hough_thresh_.get() || proba.second < hough_thresh_.get()) {
        return m;
    }

    m.gate.emplace_back();
    m.gate.front().left = left.to_msg();
    m.gate.front().right = right.to_msg();


    return m;
}
