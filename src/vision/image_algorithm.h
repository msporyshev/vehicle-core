#pragma once

#include <string>

#include <config_reader/yaml_reader.h>

#include "image_pipeline.h"
#include "objects.h"
#include "common.h"

class BinarizerHSV: public ImageProcessor
{
public:
    BinarizerHSV(const YamlReader& cfg, bool invert = false): ImageProcessor(cfg), invert_(invert) {}

    cv::Mat process(const cv::Mat& frame) const override;

    std::string name() const override { return "hsv_binary"; }
protected:

    bool invert_ = false;
    AUTOPARAM_OPTIONAL(int, h_min_, 0);
    AUTOPARAM_OPTIONAL(int, h_max_, 255);
    AUTOPARAM_OPTIONAL(int, s_min_, 0);
    AUTOPARAM_OPTIONAL(int, s_max_, 255);
    AUTOPARAM_OPTIONAL(int, v_min_, 0);
    AUTOPARAM_OPTIONAL(int, v_max_, 255);
};

class HistEqualizer: public ImageProcessor
{
public:
    HistEqualizer(const YamlReader& cfg): ImageProcessor(cfg) {}

    cv::Mat process(const cv::Mat& frame) const override;

    std::string name() const override { return "hist_equalizer"; }
protected:

    AUTOPARAM_OPTIONAL(int, channels_count_, -1);
    AUTOPARAM_OPTIONAL(double, unused_, 0.0);
};

class GrayScale: public ImageProcessor
{
public:
    cv::Mat process(const cv::Mat& frame) const override;
    std::string name() const override { return "grayscale"; }
};

class DistanceTransform: public ImageProcessor
{
public:
    cv::Mat process(const cv::Mat& frame) const override
    {
        cv::Mat result;
        cv::distanceTransform(frame, result, CV_DIST_L2, 3);
        normalize(result, result, 0, 255, cv::NORM_MINMAX, CV_8UC1);
        return result;
    }

    std::string name() const override { return "distance_transform"; }
};

class Watershed: public ImageProcessor
{
public:
    cv::Mat process(const cv::Mat& frame) const override
    {
        cv::Mat result;
        cv::watershed(frame, result);
        return result;
    }

    std::string name() const override { return "watershed"; }
};

class Threshold: public ImageProcessor
{
public:
    Threshold(const YamlReader& cfg): ImageProcessor(cfg) {}

    cv::Mat process(const cv::Mat& frame) const override
    {
        cv::Mat result;
        cv::threshold(frame, result, thresh_.get(), maxval_.get(), cv::THRESH_BINARY);
        return result;
    }

    std::string name() const override { return "threshold"; }
protected:
    AUTOPARAM_OPTIONAL(double, thresh_, 0);
    AUTOPARAM_OPTIONAL(int, maxval_, 255);
};

class DilateSquare: public ImageProcessor
{
public:
    DilateSquare(const YamlReader& cfg): ImageProcessor(cfg) {}

    cv::Mat process(const cv::Mat& frame) const override
    {
        cv::Mat result;
        cv::Mat kernel = cv::Mat::ones(side_.get(), side_.get(), CV_8UC1);
        cv::dilate(frame, result, kernel);
        return result;
    }

    std::string name() const override { return "dilate"; }
protected:
    AUTOPARAM_OPTIONAL(int, side_, 3);
};

class Invert: public ImageProcessor
{
public:
    cv::Mat process(const cv::Mat& frame) const override
    {
        cv::Mat result;
        cv::bitwise_not(frame, result);
        return result;
    }

    std::string name() const override { return "invert"; }
};


class MostCommonFilter: public ImageProcessor
{
public:
    MostCommonFilter(const YamlReader& cfg): ImageProcessor(cfg) {}

    cv::Mat process(const cv::Mat& frame) const override;
    std::string name() const override { return "most_common"; }
protected:
    AUTOPARAM_OPTIONAL(double, most_common_part_, 1.0);
};


class ApplyMask: public ImageProcessor
{
public:
    ApplyMask(const cv::Mat& mask): mask_(mask) {}

    cv::Mat process(const cv::Mat& frame) const override
    {
        cv::Mat result;
        frame.copyTo(result, mask_);
        return result;
    }

    std::string name() const override { return "apply_mask"; }
protected:
    cv::Mat mask_;
};

class ApplyMaskTo: public ImageProcessor
{
public:
    ApplyMaskTo(const cv::Mat& source): source_(source) {}

    cv::Mat process(const cv::Mat& frame) const override
    {
        cv::Mat result;
        source_.copyTo(result, frame);
        return result;
    }

    std::string name() const override { return "apply_mask_to"; }
protected:
    cv::Mat source_;
};

class MedianFilter: public ImageProcessor
{
public:
    MedianFilter(const YamlReader& cfg): ImageProcessor(cfg) {}

    cv::Mat process(const cv::Mat& frame) const override;
    std::string name() const override { return "median_filter"; }
protected:
    AUTOPARAM_OPTIONAL(int, ksize_, 5);
};

class GaussianFilter: public ImageProcessor
{
public:
    GaussianFilter(const YamlReader& cfg): ImageProcessor(cfg) {}

    cv::Mat process(const cv::Mat& frame) const override;
    std::string name() const override { return "gaussian_filter"; }
protected:
    AUTOPARAM_OPTIONAL(int, kx_, 3);
    AUTOPARAM_OPTIONAL(int, ky_, 3);
    AUTOPARAM_OPTIONAL(int, sigma_x_, 0);
    AUTOPARAM_OPTIONAL(int, sigma_y_, 0);
};

class AbsDiffFilter: public ImageProcessor
{
public:
    AbsDiffFilter(const YamlReader& cfg, const cv::Mat& source)
            : ImageProcessor(cfg)
            , source_(source)
    {}

    cv::Mat process(const cv::Mat& frame) const override;
    std::string name() const override { return "absdiff_filter"; }
protected:
    cv::Mat source_;
};

class SobelFilter: public ImageProcessor
{
public:
    SobelFilter(const YamlReader& cfg): ImageProcessor(cfg) {}

    cv::Mat process(const cv::Mat& frame) const override;
    std::string name() const override { return "sobel"; }
protected:
    AUTOPARAM_OPTIONAL(int, dx_, 0);
    AUTOPARAM_OPTIONAL(int, dy_, 0);
    AUTOPARAM_OPTIONAL(int, ddepth_, -1);
    AUTOPARAM_OPTIONAL(int, ksize_, 3);
};

class LaplacianFilter: public ImageProcessor
{
public:
    LaplacianFilter(const YamlReader& cfg): ImageProcessor(cfg) {}

    cv::Mat process(const cv::Mat& frame) const override;
    std::string name() const override { return "laplacian"; }
protected:
    AUTOPARAM_OPTIONAL(int, ksize_, 3);
};

class FrameDrawer: public ImageProcessor
{
public:
    FrameDrawer(const YamlReader& cfg): ImageProcessor(cfg) {}

    cv::Mat process(const cv::Mat& frame) const override;
    std::string name() const override { return "frame_drawer"; }
protected:
    AUTOPARAM_OPTIONAL(int, width_, 5);
    AUTOPARAM_OPTIONAL(int, color_, 0);
};

class ObjectDrawer: public ImageProcessor
{
public:
    ObjectDrawer(const YamlReader& cfg, const std::vector<cv::Point>& object)
            : ImageProcessor(cfg)
            , objects_({object})
    {}

    ObjectDrawer(const YamlReader& cfg, const std::vector<std::vector<cv::Point>>& objects)
            : ImageProcessor(cfg)
            , objects_(objects)
    {}

    cv::Mat process(const cv::Mat& frame) const override;
    std::string name() const override { return "object_drawer"; }

protected:
    AUTOPARAM_OPTIONAL(int, width_, 2);
    AUTOPARAM_OPTIONAL(std::string, color_, "orange");
    std::vector<std::vector<cv::Point>> objects_;
};

class FindContours: public Processor<std::vector<Contour>, cv::Mat>
{
public:
    FindContours(const YamlReader& cfg): cfg_(cfg) {}

    std::vector<Contour> process(const cv::Mat& image) const override;

private:
    YamlReader cfg_;

    AUTOPARAM_OPTIONAL(int, approx_dist_, 0);
    AUTOPARAM_OPTIONAL(int, max_approx_count_, 1e9);
    AUTOPARAM_OPTIONAL(int, min_approx_count_, 0);
};

class FilterSimilarContours: public Processor<std::vector<Contour>, std::vector<Contour>>
{
public:
    FilterSimilarContours(const YamlReader& cfg): cfg_(cfg) {}

    std::vector<Contour> process(const std::vector<Contour>& contours) const override;
private:
    YamlReader cfg_;

    AUTOPARAM(double, accuracy_);
};

class MinMaxStripes: public Processor<std::vector<Stripe>, std::vector<Contour>>
{
public:
    MinMaxStripes(const YamlReader& cfg): cfg_(cfg) {}

    std::vector<Stripe> process(const std::vector<Contour>& contours) const override;

private:
    YamlReader cfg_;
};

class AllStripes: public Processor<std::vector<Stripe>, cv::Mat>
{
public:
    AllStripes(const YamlReader& cfg): cfg_(cfg) {}
    std::vector<Stripe> process(const cv::Mat& frame) const override;

private:
    YamlReader cfg_;

    AUTOPARAM(bool, enable_col_cor_);
};

class FilterStripes: public Processor<std::vector<Stripe>, std::vector<Stripe> >
{
public:
    FilterStripes(const YamlReader& cfg): cfg_(cfg) {}

    std::vector<Stripe> process(const std::vector<Stripe>& stripes) const override;
private:
    YamlReader cfg_;

    AUTOPARAM_OPTIONAL(int, max_length_, 1e9);
    AUTOPARAM_OPTIONAL(int, min_length_, 0);
    AUTOPARAM_OPTIONAL(int, max_width_, 1e9);
    AUTOPARAM_OPTIONAL(int, min_width_, 0);
    AUTOPARAM_OPTIONAL(double, min_angle_, -1e9);
    AUTOPARAM_OPTIONAL(double, max_angle_, 1e9);
    AUTOPARAM_OPTIONAL(double, min_sides_ratio_, 0);
    AUTOPARAM_OPTIONAL(double, max_sides_ratio_, 1e9);
};


