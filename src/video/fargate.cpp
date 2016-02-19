#include <video/MsgFoundGate.h>
#include <config_reader/yaml_reader.h>
#include <video/MsgStripe.h>
#include <video/MsgFoundStripe.h>
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
    boost::optional<video::MsgFoundStripe> find(const cv::Mat& frame, cv::Mat& out, Mode mode)
    {
        boost::optional<video::MsgFoundStripe> result;

        ImagePipeline pipe;
        pipe
            << MedianFilter(cfg_.node("median_big"))
            << AbsDiffFilter(cfg_, frame)
            << GrayScale()
            << Threshold(cfg_.node("thresh1"))
            << DistanceTransform()
            << MedianFilter(cfg_.node("median"))
            << Threshold(cfg_.node("thresh2"))
            ;
        out = pipe.process(frame);

        FindContours cont_finder(cfg_.node("contours"));
        MinMaxStripes stripe_transform(cfg_);

        auto contours = cont_finder.process(out);
        auto stripes = stripe_transform.process(contours);

        if (stripes.empty()) {
            return result;
        }

        video::MsgFoundStripe m;
        int stripes_count = stripes.size();
        for (const auto& stripe : stripes) {
            video::MsgStripe s;
            s.begin = MakePoint2(stripe.line.first.x, stripe.line.first.y);
            s.end = MakePoint2(stripe.line.second.x, stripe.line.second.y);
            s.wbegin = MakePoint2(stripe.width.first.x, stripe.width.first.y);
            s.wend = MakePoint2(stripe.width.second.x, stripe.width.second.y);
            s.width = norm(s.wbegin - s.wend);
            m.stripes.push_back(s);
        }

        return m;
    }

private:
    YamlReader cfg_;
};

REGISTER_RECOGNIZER(FarGateRecognizer, fargate);
