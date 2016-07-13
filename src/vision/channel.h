#include <vision/MsgFoundGate.h>
#include <config_reader/yaml_reader.h>
#include <point/point.h>

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <vector>

#include "common.h"
#include "objects.h"
#include "rec_factory.h"
#include "image_pipeline.h"
#include "image_algorithm.h"

using namespace std;

class ChannelRecognizer
{
public:
    ChannelRecognizer(const YamlReader& cfg) : cfg_(cfg) {}
    boost::optional<vision::MsgFoundGate> find(const cv::Mat& frame, cv::Mat& out, Mode mode);
    boost::optional<Stripe> find_stripe(const cv::Mat& frame, cv::Mat& out, int x);

private:
    YamlReader cfg_;

    AUTOPARAM(int, border_delta_);
    AUTOPARAM(bool, enable_correction_);
};

REGISTER_RECOGNIZER(ChannelRecognizer, channel);
