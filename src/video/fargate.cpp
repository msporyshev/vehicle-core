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

        ImagePipeline pipe(mode);
        pipe
            << MedianFilter(cfg_.node("median_big"))
            << AbsDiffFilter(cfg_, frame)
            << GrayScale()
            // << Threshold(cfg_.node("thresh1"))
            // << DistanceTransform()
            // << MedianFilter(cfg_.node("median"))
            // << Threshold(cfg_.node("thresh2"))
            ;
        // out = pipe.process(frame);


        Pipeline<std::vector<Contour>, cv::Mat> p;
        p << pipe;
        p << FindContours(cfg_.node("contours"));
        auto res = p.process(frame);
        // FindContours cont_finder(cfg_.node("contours"));
        // MinMaxStripes stripe_transform(cfg_);

        // auto contours = cont_finder.process(out);
        // auto stripes = stripe_transform.process(contours);

        // if (stripes.empty()) {
        //     return result;
        // }

        video::MsgFoundStripe m;
        // int stripes_count = stripes.size();
        // for (const auto& stripe : stripes) {
        //     m.stripes.push_back(stripe.to_msg());
        // }

        return m;
    }

private:
    YamlReader cfg_;
};

REGISTER_RECOGNIZER(FarGateRecognizer, fargate);
