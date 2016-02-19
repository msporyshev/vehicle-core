#include "image_algorithm.h"

#include <opencv2/opencv.hpp>
#include <vector>

#include <vector>

using namespace cv;
using namespace std;

namespace { // namespace

Mat equalize_channel(Mat src, double unused) {
    Mat res = src.clone();

    vector<double> hist(256);

    int all_count = src.rows * src.cols;

    int left_bound = 1000;
    int right_bound = 0;

    for (int i = 0; i < src.rows; i++) {
        for (int j = 0; j < src.cols; j++) {
            int val = src.at<uchar>(i, j);
            hist[val]+=1;
            right_bound = max(right_bound, val);
            left_bound = min(left_bound, val);
        }
    }

    int need_count = all_count * unused;
    int left_count = 0;
    int right_count = 0;
    int k = 0;
    while(left_count < need_count) {
        left_count += hist[k];
        k++;
    }
    left_bound = max(left_bound, k);
    k = hist.size() - 1;
    while(right_count < need_count) {
        right_count += hist[k];
        k--;
    }
    right_bound = min(right_bound, k);
    all_count -= left_count + right_count;

    if (right_bound == left_bound) {
        return res;
    }

    for (int i = 0; i < src.rows; i++) {
        for (int j = 0; j < src.cols; j++) {
            int oldval = src.at<uchar>(i, j);
            int newval = oldval;

            newval = 255.0 * (oldval - left_bound) / (right_bound - left_bound);
            newval = min(newval, 255);
            newval = max(newval, 0);
            res.at<uchar>(i, j) = newval;
        }
    }

    return res;
}

} // namespace


cv::Mat BinarizerHSV::process(const cv::Mat& frame)
{
    Mat hsv;
    cvtColor(frame, hsv, CV_BGR2HSV);
    vector<Mat> channels(3);
    split(hsv, channels);

    Mat bin_h, bin_s, bin_v, bin_img;
    int hmin = h_min_.get();
    int hmax = h_max_.get();

    if (hmin >= 0) { //если отрицательное число, то это на самом деле оборот через 180 в H-канале
        inRange(channels[0], hmin, hmax, bin_h);
    } else {
        Mat range1, range2;
        inRange(channels[0], 180+hmin, 180, range1);
        inRange(channels[0], 0, hmax, range2);
        bitwise_or(range1, range2, bin_h);
    }

    int smin = s_min_.get();
    int smax = s_max_.get();

    int vmin = v_min_.get();
    int vmax = v_max_.get();

    inRange(channels[1], smin, smax, bin_s);
    inRange(channels[2], vmin, vmax, bin_v);

    bitwise_and(bin_s, bin_h, bin_img);
    bitwise_and(bin_v, bin_img, bin_img);

    return bin_img;
}

cv::Mat HistEqualizer::process(const cv::Mat& frame)
{
    vector<Mat> channels, channels_eq;
    split(frame, channels);

    channels_eq = channels;

    int channels_count = channels_count_.is_set() ? channels_count_.get() : channels.size();

    for (int i = 0; i < channels_count; i++) {
        channels_eq[i] = equalize_channel(channels[i], unused_.get());
    }

    Mat equalized;
    merge(channels_eq, equalized);

    return equalized;
}

cv::Mat GrayScale::process(const cv::Mat& frame)
{
    Mat result;
    cvtColor(frame, result, CV_BGR2GRAY);
    return result;
}


cv::Mat MedianFilter::process(const cv::Mat& frame)
{
    Mat result;
    medianBlur(frame, result, ksize_.get());
    return result;
}

cv::Mat GaussianFilter::process(const cv::Mat& frame)
{
    Mat result;
    GaussianBlur(frame, result, Size(kx_.get(), ky_.get()),
        sigma_x_.get(), sigma_y_.get());
    return result;
}

cv::Mat AbsDiffFilter::process(const cv::Mat& frame)
{
    Mat result;
    cv::absdiff(source_, frame, result);
    return result;
}

cv::Mat SobelFilter::process(const cv::Mat& frame)
{
    Mat result;
    Sobel(frame, result, ddepth_.get(), dx_.get(), dy_.get(), ksize_.get());
    return result;
}

cv::Mat LaplacianFilter::process(const cv::Mat& frame)
{
    Mat result;
    Laplacian(frame, result, -1, ksize_.get());
    return result;
}

cv::Mat FrameDrawer::process(const cv::Mat& frame)
{
    Mat result = frame.clone();

    int color = color_.get();
    int width = width_.get();

    for (int i = 0; i < result.rows; i++) {
        for (int j = 0; j < width; j++) {
            result.at<uchar>(i, j) = color;
            result.at<uchar>(i, result.cols - j - 1) = color;
        }
    }

    for (int i = 0; i < result.cols; i++) {
        for (int j = 0; j < width; j++) {
            result.at<uchar>(j, i) = color;
            result.at<uchar>(result.rows - j - 1, i) = color;
        }
    }

    return result;
}

cv::Mat ObjectDrawer::process(const cv::Mat& frame)
{
    Mat result = frame.clone();

    int width = width_.get();
    std::string color_name = color_.get();

    for (const auto& obj : objects_) {
        for (size_t i = 1; i < obj.size(); ++i) {
            cv::line(result, obj[i - 1], obj[i], scalar_by_color.at(color_by_name.at(color_name)),
                width);
        }
    }

    return result;
}

vector<vector<Point>> FindContours::process(const Mat& image)
{
    std::vector<std::vector<cv::Point>> contours, approxes;
    std::vector<cv::Vec4i> hierarchy;

    findContours(image, contours, hierarchy,
        CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

    if (!approx_dist_.is_set()) {
        return contours;
    }

    for (size_t i = 0; i < contours.size(); i++) {
        std::vector<cv::Point> approx;
        approxPolyDP(contours[i], approx, approx_dist_.get(), false);
        if (approx.size() < min_approx_count_.get() || approx.size() > max_approx_count_.get()) {
            continue;
        }

        approxes.push_back(approx);
    }

    return approxes;
}