#pragma once

#include <string>
#include <unordered_map>
#include <memory>
#include <type_traits>

#include <config_reader/yaml_reader.h>

#include <opencv2/opencv.hpp>

#include "common.h"
#include "objects.h"

template<typename T>
inline void draw_debug(const T& obj, cv::Mat& debug)
{
    obj.draw(debug, Color::Yellow, 2);
}

template<>
inline void draw_debug<cv::Mat>(const cv::Mat& obj, cv::Mat& debug)
{
    debug = obj.clone();
}

template<>
inline void draw_debug<std::vector<Contour>>(const std::vector<Contour>& obj, cv::Mat& debug)
{
    for (const auto& contour : obj) {
        draw_debug(contour, debug);
    }
}

template<>
inline void draw_debug<std::vector<Stripe>>(const std::vector<Stripe>& obj, cv::Mat& debug)
{
    for (const auto& contour : obj) {
        draw_debug(contour, debug);
    }
}

template<typename T>
inline void show_image(const std::string& title, const T& obj, cv::Mat& frame)
{
    cv::Mat debug = frame.clone();
    draw_debug(obj, debug);
    imshow(title, debug);
}

template<typename Out, typename In>
class Processor
{
public:
    Processor(Mode mode = Mode::Onboard): mode_(mode) {}

    using OutType = Out;
    using InType = In;
    using Ptr = std::shared_ptr<Processor>;

    virtual Out process(const In&) const = 0;

    virtual std::string name() const { return "processor"; }

    const Mode& mode() const { return mode_; }

protected:
    Mode mode_;
};

template<typename In, typename Out>
class FuncWrapper
{
public:
    using ImageFunc = std::function<Out (const In&, cv::Mat&, Mode)>;

    FuncWrapper() {}
    FuncWrapper(ImageFunc func, std::string name = ""): func_(func), name_(name) {}

    inline Out process(const In& in, cv::Mat& debug, Mode mode) const
    {
        return func_(in, debug, mode);
    }
private:
    ImageFunc func_;
    std::string name_;
};

template<typename ProcessorA, typename ProcessorB>
FuncWrapper<typename ProcessorA::InType, typename ProcessorB::OutType> operator+(
        ProcessorA a, ProcessorB b)
{
    static_assert(std::is_same<typename ProcessorA::OutType, typename ProcessorB::InType>::value,
        "Processor types mismatch in a binary expression");
    typedef FuncWrapper<typename ProcessorA::InType, typename ProcessorB::OutType> Func;

    return Func([a, b](const typename ProcessorA::InType& in, cv::Mat& debug, Mode mode) {
        auto ares = a.process(in);
        auto bres = b.process(ares);
        if (mode == Mode::Debug) {
            show_image(a.name(), ares, debug);
            show_image(b.name(), bres, debug);
        }
        return bres;
    });
}


template<typename In, typename Mid, typename Processor>
FuncWrapper<In, typename Processor::OutType> operator+(
        FuncWrapper<In, Mid> a, Processor b)
{
    static_assert(std::is_same<Mid, typename Processor::InType>::value,
        "Processor types mismatch in a binary expression");

    typedef FuncWrapper<In, typename Processor::OutType> Func;

    return Func([a, b](const In& in, cv::Mat& debug, Mode mode) {
        auto ares = a.process(in, debug, mode);
        auto bres = b.process(ares);
        if (mode == Mode::Debug) {
            show_image(b.name(), bres, debug);
        }
        return bres;
    });
}

template<typename In, typename Mid, typename Out>
FuncWrapper<In, Out> operator+(FuncWrapper<In, Mid> a, FuncWrapper<Mid, Out> b)
{
    return FuncWrapper<In, Out>([a, b](const In& in, cv::Mat& debug, Mode mode) {
        auto ares = a.process(in, debug, mode);
        auto bres = b.process(ares, debug, mode);
        return bres;
    });
}

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

    cv::Mat process(const cv::Mat& frame) const
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
    Out process(const In& data) const
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
    T process(const T& data) const override
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
    ImagePipeline(Mode mode = Mode::Debug, std::string name = "image_pipeline")
            : mode_(mode), name_(name) {}

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

    cv::Mat process(const cv::Mat& frame) const override
    {
        cv::Mat in = frame;
        std::unordered_map<std::string, int> cur_name_count;

        for (auto& processor : processors_) {
            cv::Mat out = processor->process(in);

            if (mode_ == Mode::Debug) {
                cv::Mat debug = out.clone();

                std::string title = name() + "_" + processor->name();
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


