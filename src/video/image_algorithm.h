#pragma once

#include <string>

#include <config_reader/yaml_reader.h>

#include "image_pipeline.h"
#include "common.h"

class BinarizerHSV: public ImageProcessor
{
public:
    BinarizerHSV(const YamlReader& cfg): ImageProcessor(cfg) {}

    cv::Mat process(const cv::Mat& frame) override;

    std::string name() const override { return "hsv_binary"; }
protected:

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

    cv::Mat process(const cv::Mat& frame) override;

    std::string name() const override { return "hist_equalizer"; }
protected:

    AUTOPARAM_OPTIONAL(int, channels_count_, -1);
    AUTOPARAM_OPTIONAL(double, unused_, 0.0);
};

class GrayScale: public ImageProcessor
{
public:
    cv::Mat process(const cv::Mat& frame) override;
    std::string name() const override { return "grayscale"; }
};

class MedianBlur: public ImageProcessor
{
public:
    MedianBlur(const YamlReader& cfg): ImageProcessor(cfg) {}

    cv::Mat process(const cv::Mat& frame) override;
    std::string name() const override { return "median_blur"; }
protected:
    AUTOPARAM_OPTIONAL(int, ksize_, 5);
};

class SobelFilter: public ImageProcessor
{
public:
    SobelFilter(const YamlReader& cfg): ImageProcessor(cfg) {}

    cv::Mat process(const cv::Mat& frame) override;
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

    cv::Mat process(const cv::Mat& frame) override;
    std::string name() const override { return "laplacian"; }
protected:
    AUTOPARAM_OPTIONAL(int, ksize_, 3);
};

class FrameDrawer: public ImageProcessor
{
public:
    FrameDrawer(const YamlReader& cfg): ImageProcessor(cfg) {}

    cv::Mat process(const cv::Mat& frame) override;
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

    cv::Mat process(const cv::Mat& frame) override;
    std::string name() const override { return "object_drawer"; }

protected:
    AUTOPARAM_OPTIONAL(int, width_, 2);
    AUTOPARAM_OPTIONAL(std::string, color_, "orange");
    std::vector<std::vector<cv::Point>> objects_;
};

class FindContours
{
public:
    FindContours(const YamlReader& cfg): cfg_(cfg) {}

    std::vector<std::vector<cv::Point>> process(const cv::Mat& image);

private:
    YamlReader cfg_;

    AUTOPARAM_OPTIONAL(int, approx_dist_, 0);
    AUTOPARAM_OPTIONAL(int, max_approx_count_, 1e9);
    AUTOPARAM_OPTIONAL(int, min_approx_count_, 0);
};

