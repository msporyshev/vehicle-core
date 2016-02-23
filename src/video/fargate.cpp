#include <video/MsgFoundGate.h>
#include <config_reader/yaml_reader.h>
#include <point/point.h>

#include <opencv2/opencv.hpp>

#include <algorithm>

#include "common.h"
#include "rec_factory.h"
#include "image_pipeline.h"
#include "image_algorithm.h"

class FarGateRecognizer
{
public:
    FarGateRecognizer(const YamlReader& cfg) : cfg_(cfg) {}
    boost::optional<video::MsgFoundGate> find(const cv::Mat& frame, cv::Mat& out, Mode mode)
    {
        boost::optional<video::MsgFoundGate> result;

        ImagePipeline lanes_filter(mode);
        lanes_filter
            << BinarizerHSV(cfg_.node("hsv"), true)
            ;

        auto lanes_mask = lanes_filter.process(frame);

        ImagePipeline pipe(mode);
        pipe
            << MedianFilter(cfg_.node("median_big"))
            << AbsDiffFilter(cfg_, frame)
            << GrayScale()
            << Threshold(cfg_.node("thresh1"))
            << DistanceTransform()
            << Threshold(cfg_.node("thresh2"))
            // << ApplyMask(lanes_mask)
            << MedianFilter(cfg_.node("median"))
            ;
        auto preprocessed = pipe.process(frame);


        FindContours cont_finder(cfg_);
        MinMaxStripes stripe_transform(cfg_);
        FilterStripes filter_stripes(cfg_);

        auto contours = cont_finder.process(preprocessed);
        auto raw_stripes = stripe_transform.process(contours);
        auto stripes = filter_stripes.process(raw_stripes);


        cv::Mat debug = frame.clone();
        for (auto& stripe : stripes) {
            stripe.draw(out, Color::Orange, 2);
        }

        sort(stripes.begin(), stripes.end(), [](const Stripe& a, const Stripe& b) { return a.len() > b.len(); });

        if (stripes.size() < 2) {
            return result;
        }

        video::MsgFoundGate m;
        m.gate.left = stripes[0].to_msg();
        m.gate.right = stripes[1].to_msg();

        return m;
    }

private:
    YamlReader cfg_;
};

REGISTER_RECOGNIZER(FarGateRecognizer, fargate);
