#pragma once

#include <string>

#include <config_reader/yaml_reader.h>

#include "image_pipeline.h"

class BinarizerHSV: public ImageProcessor
{
public:
    BinarizerHSV(const YamlReader& cfg): ImageProcessor(cfg) {}

    void process(const cv::Mat& frame, cv::Mat& result) override;

    std::string name() const override { return "hsv_binary"; }
protected:

    AUTOPARAM(int, h_min_);
    AUTOPARAM(int, h_max_);
    AUTOPARAM(int, s_min_);
    AUTOPARAM(int, s_max_);
    AUTOPARAM(int, v_min_);
    AUTOPARAM(int, v_max_);
};