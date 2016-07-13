#include <vision/MsgFoundGate.h>
#include <config_reader/yaml_reader.h>
#include <point/point.h>

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <vector>

#include "common.h"
#include "rec_factory.h"
#include "image_pipeline.h"
#include "image_algorithm.h"

using namespace std;

class FarGateRecognizer
{
public:
    FarGateRecognizer(const YamlReader& cfg) : cfg_(cfg) {}
    boost::optional<vision::MsgFoundGate> find(const cv::Mat& frame, cv::Mat& out, Mode mode);

private:

    YamlReader cfg_;

    AUTOPARAM(int, min_gate_width_);
    AUTOPARAM(double, hough_thresh_);
    AUTOPARAM_OPTIONAL(int, cell_pixels_, 1);
};

REGISTER_RECOGNIZER(FarGateRecognizer, fargate);
