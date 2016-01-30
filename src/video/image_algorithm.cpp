#include "image_algorithm.h"

#include <opencv2/opencv.hpp>

using namespace cv;

void BinarizerHSV::process(const cv::Mat& frame, cv::Mat& result)
{
    result = frame.clone();

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

    result = bin_img;
}

void GrayScale::process(const cv::Mat& frame, cv::Mat& result)
{
    cvtColor(frame, result, CV_BGR2GRAY);
}


void MedianBlur::process(const cv::Mat& frame, cv::Mat& result)
{
    medianBlur(frame, result, ksize_.get());
}

void FrameDrawer::process(const cv::Mat& frame, cv::Mat& result)
{
    result = frame.clone();

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
}