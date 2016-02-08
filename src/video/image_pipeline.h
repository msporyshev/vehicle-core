#pragma once

#include <string>
#include <unordered_map>
#include <memory>
#include <type_traits>

#include <config_reader/yaml_reader.h>

#include <opencv2/opencv.hpp>

#include "common.h"

class ImageProcessor
{
public:
    ImageProcessor() {}
    ImageProcessor(const YamlReader& cfg): cfg_(cfg) {}

    virtual void process(const cv::Mat& frame, cv::Mat& result) = 0;

    virtual std::string name() const { return "processor"; }

protected:
    YamlReader cfg_;
};

template<typename Func>
class SimpleFunc: public ImageProcessor
{
public:
    SimpleFunc(Func func): ImageProcessor(), func_(func) {}

    void process(const cv::Mat& frame, cv::Mat& result)
    {
        func_(frame, result);
    }
private:
    Func func_;
};

template<typename Func>
SimpleFunc<Func> simple_func(Func func)
{
    return SimpleFunc<Func>(func);
}

class ImagePipeline
{
public:
    ImagePipeline(Mode mode = Mode::Debug): mode_(mode) {}

    template<typename Processor>
    typename std::enable_if<std::is_base_of<ImageProcessor, Processor>::value, ImagePipeline&>::type operator<<(Processor&& processor)
    {
        processors_.emplace_back(std::make_shared<Processor>(processor));

        return *this;
    }

    template<typename Func>
    typename std::enable_if<!std::is_base_of<ImageProcessor, Func>::value, ImagePipeline&>::type operator<<(Func func)
    {

        processors_.emplace_back(std::make_shared<SimpleFunc<Func> >(func));
        return *this;
    }

    cv::Mat process(const cv::Mat& frame)
    {
        cv::Mat in = frame;
        cv::Mat out = in.clone();

        std::unordered_map<std::string, int> cur_name_count;


        for (auto& processor : processors_) {
            processor->process(in, out);

            if (mode_ == Mode::Debug) {
                cv::Mat debug = out.clone();

                std::string title = processor->name();
                int count = cur_name_count[title]++;

                if (count > 0) {
                    title += " " + std::to_string(count);
                }

                imshow(title, debug);
            }

            swap(in, out);
        }

        return in;
    }

private:
    std::vector<std::shared_ptr<ImageProcessor> > processors_;


    Mode mode_ = Mode::Debug;
};

