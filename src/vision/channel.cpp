#include "channel.h"
#include "fargate.h"
#include "objects.h"

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

    double mean = 0;

    for (int i = 0; i < xcount.size(); i++) {
        mean += i * xcount[i];
    }
    mean /= all_count;

    double stddev = 0;
    for (int i = 0; i < xcount.size(); i++) {
        stddev += (mean - i) * (mean - i) / all_count;
    }
    stddev = sqrt(stddev);

    mean *= cell_pixels;
    stddev *= cell_pixels;



    // double proba(1.0 * first_peak / cell_pixels / bin.rows,
        // 1.0 * second_peak / cell_pixels / bin.rows);

    // ROS_INFO_STREAM("Peak proba: " << proba.first << " " << proba.second);

    Stripe left(Segment(cv::Point2d(mean, 0), cv::Point2d(mean, 400)),
        Segment(cv::Point2d(mean - stddev / 2, 200), cv::Point2d(mean + stddev / 2, 200)));
    // Stripe right(Segment(cv::Point2d(second_peak_x, 0), cv::Point2d(second_peak_x, 400)));

    // Color color = proba.first > hough_thresh_.get() ? Color::Orange : Color::Green;
    left.draw(out, Color::Orange, 2);


    // if (proba.first < hough_thresh_.get() || proba.second < hough_thresh_.get()) {
    //     return m;
    // }

    vision::MsgFoundGate m;
    m.gate.emplace_back();
    m.gate.front().left = left.to_msg();



    return m;
}