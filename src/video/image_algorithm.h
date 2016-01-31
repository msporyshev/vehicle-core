#pragma once

#include <string>

#include <config_reader/yaml_reader.h>

#include "image_pipeline.h"
#include "common.h"

class BinarizerHSV: public ImageProcessor
{
public:
    BinarizerHSV(const YamlReader& cfg): ImageProcessor(cfg) {}

    void process(const cv::Mat& frame, cv::Mat& result) override;

    std::string name() const override { return "hsv_binary"; }
protected:

    AUTOPARAM_OPTIONAL(int, h_min_, 0);
    AUTOPARAM_OPTIONAL(int, h_max_, 255);
    AUTOPARAM_OPTIONAL(int, s_min_, 0);
    AUTOPARAM_OPTIONAL(int, s_max_, 255);
    AUTOPARAM_OPTIONAL(int, v_min_, 0);
    AUTOPARAM_OPTIONAL(int, v_max_, 255);
};

class GrayScale: public ImageProcessor
{
public:
    void process(const cv::Mat& frame, cv::Mat& result) override;
    std::string name() const override { return "grayscale"; }
};

class MedianBlur: public ImageProcessor
{
public:
    MedianBlur(const YamlReader& cfg): ImageProcessor(cfg) {}

    void process(const cv::Mat& frame, cv::Mat& result) override;
    std::string name() const override { return "median_blur"; }
protected:
    AUTOPARAM_OPTIONAL(int, ksize_, 5);
};


class FrameDrawer: public ImageProcessor
{
public:
    FrameDrawer(const YamlReader& cfg): ImageProcessor(cfg) {}

    void process(const cv::Mat& frame, cv::Mat& result) override;
    std::string name() const override { return "median_blur"; }
protected:
    AUTOPARAM_OPTIONAL(int, width_, 5);
    AUTOPARAM_OPTIONAL(int, color_, 0);
};