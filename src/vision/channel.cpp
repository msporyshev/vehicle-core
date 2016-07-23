#include "channel.h"
#include "fargate.h"
#include "objects.h"

#include <algorithm>

using namespace std;

boost::optional<vision::MsgFoundGate> ChannelRecognizer::find(const cv::Mat& frame, cv::Mat& out, Mode mode)
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
        << MedianFilter(cfg_.node("median"))
        // << MedianFilter(cfg_.node("median_big"))
        // << AbsDiffFilter(cfg_, filtered)
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

    int max_count = *max_element(xcount.begin(), xcount.end());

    if (mode == Mode::Debug) {
        cv::Mat hist_frame = frame.clone();
        Hist hist(xcount);
        hist.draw(hist_frame, Color::Orange);
        cv::imshow("histogram", hist_frame);
    }

    vector<int> filtered_hist = xcount;
    int filtered_sum = 0;
    for (auto& elem : filtered_hist) {
        // cout << "elem: " << elem << " max_count: " << max_count << endl;
        if (elem < max_count * noise_thresh_.get()) {
            elem = 0;
        }
        filtered_sum += elem;
    }

    if (mode == Mode::Debug) {
        cv::Mat hist_frame = frame.clone();
        Hist hist(filtered_hist);
        hist.draw(hist_frame, Color::Orange);
        cv::imshow("filtered histogram", hist_frame);
    }

    double mean = 0;

    for (int i = 0; i < filtered_hist.size(); i++) {
        mean += i * filtered_hist[i];
    }
    mean /= filtered_sum;

    double stddev = 0;
    for (int i = 0; i < filtered_hist.size(); i++) {
        stddev += filtered_hist[i] * (mean - i) * (mean - i) / filtered_sum;
    }
    stddev = sqrt(stddev);

    mean *= cell_pixels;
    stddev *= cell_pixels;

    double width = 2 * stddev;

    cout << "center: " << mean << " width: " << width << endl;

    Stripe left(Segment(cv::Point2d(mean, 0), cv::Point2d(mean, 400)),
        Segment(cv::Point2d(mean - stddev, 200), cv::Point2d(mean + stddev, 200)));

    left.draw(out, Color::Orange, 2);

    vision::MsgFoundGate m;
    m.gate.emplace_back();
    m.gate.front().left = left.to_msg();

    return m;
}