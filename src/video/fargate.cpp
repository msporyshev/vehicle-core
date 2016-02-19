#include <video/MsgFoundGate.h>
#include <config_reader/yaml_reader.h>

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
        // medianBlur(src, blured, 5);
        boost::optional<video::MsgFoundGate> result;

        ImagePipeline pipe;
        pipe
            << MedianFilter(cfg_.node("median_big"))
            << AbsDiffFilter(cfg_, frame)
            // << Watershed()
            << GrayScale()
            << Threshold(cfg_.node("thresh1"))
            << DistanceTransform()
        //     << ApplyMaskTo(src)
        //     // << BinarizerHSV(cfg_.node("binarizer"))
        //     // << FrameDrawer(cfg_)
            << MedianFilter(cfg_.node("median"))
            << Threshold(cfg_.node("thresh2"))
            ;
        out = pipe.process(frame);

        return result;
    }

private:
    YamlReader cfg_;
};

REGISTER_RECOGNIZER(FarGateRecognizer, fargate);
