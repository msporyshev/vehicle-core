#pragma once

#include <string>
#include <unordered_map>
#include <memory>
#include <type_traits>

#include <config_reader/yaml_reader.h>

#include <opencv2/opencv.hpp>

#include "common.h"

template<typename Out, typename In>
class Processor
{
public:
    using OutType = Out;
    using InType = In;
    using Ptr = std::shared_ptr<Processor>;

    virtual Out process(const In&) = 0;

    virtual std::string name() const { return "processor"; }
};

class ImageProcessor: public Processor<cv::Mat, cv::Mat>
{
public:
    ImageProcessor() {}
    ImageProcessor(const YamlReader& cfg): cfg_(cfg) {}
protected:
    YamlReader cfg_;
};

template<typename Func>
class SimpleFunc: public ImageProcessor
{
public:
    SimpleFunc(Func func): ImageProcessor(), func_(func) {}

    cv::Mat process(const cv::Mat& frame)
    {
        return func_(frame);
    }
private:
    Func func_;
};

template<typename Func>
SimpleFunc<Func> simple_func(Func func)
{
    return SimpleFunc<Func>(func);
}

template <typename Out, typename In>
class Pipeline: public Processor<Out, In>
{
public:
    Out process(const In& data)
    {
        In in_result = data;
        for (auto& p : in_pipe_) {
            in_result = p->process(in_result);
        }

        Out out_result;
        out_result = middle_->process(in_result);

        for (auto& p : out_pipe_) {
            out_result = p->process(out_result);
        }

        return out_result;
    }

    Pipeline& operator<<(typename Processor<In, In>::Ptr processor)
    {
        in_pipe_.push_back(processor);
        return *this;
    }

    Pipeline& operator<<(typename Processor<Out, Out>::Ptr processor)
    {
        out_pipe_.push_back(processor);
        return *this;
    }

    Pipeline& operator<<(typename Processor<Out, In>::Ptr processor)
    {
        middle_ = processor;
        return *this;
    }

    template<typename ProcessorType>
    Pipeline& operator<<(const ProcessorType& processor)
    {
        *this << static_cast<typename Processor<typename ProcessorType::OutType, typename ProcessorType::InType>::Ptr>(
            std::make_shared<ProcessorType>(processor));
        return *this;
    }

private:
    std::vector<typename Processor<In, In>::Ptr> in_pipe_;
    typename Processor<Out, In>::Ptr middle_;
    std::vector<typename Processor<Out, Out>::Ptr> out_pipe_;
};

template<typename T>
class Pipeline<T, T>: public Processor<T, T>
{
public:
    T process(const T& data) override
    {
        T current_data = data;
        for (auto& processor : processors_) {
            current_data = processor->process(current_data);
        }

        return current_data;
    }

    Pipeline& operator<<(typename Processor<T, T>::Ptr processor)
    {
        processors_.push_back(processor);
        return *this;
    }

    template<typename Processor>
    Pipeline& operator<<(const Processor& processor)
    {
        processors_.emplace_back(std::make_shared<Processor>(processor));
        return *this;
    }

protected:
    std::vector<typename Processor<T, T>::Ptr> processors_;
};

class ImagePipeline: public Processor<cv::Mat, cv::Mat>
{
public:
    ImagePipeline(Mode mode = Mode::Debug, std::string name_ = "image_pipeline"): mode_(mode) {}

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

    cv::Mat process(const cv::Mat& frame) override
    {
        cv::Mat in = frame;
        std::unordered_map<std::string, int> cur_name_count;

        for (auto& processor : processors_) {
            cv::Mat out = processor->process(in);

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

    std::string name() const override { return name_; }

    void clear_processors()
    {
        processors_.clear();
    }

private:
    std::vector<std::shared_ptr<ImageProcessor> > processors_;
    std::string name_;


    Mode mode_ = Mode::Debug;
};


