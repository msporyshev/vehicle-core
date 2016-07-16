#include <vision/MsgFoundBin.h>
#include <config_reader/yaml_reader.h>
#include <point/point.h>

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <vector>

#include "common.h"
#include "rec_factory.h"
#include "image_pipeline.h"
#include "image_algorithm.h"

#include <vision/MsgBin.h>

using namespace std;

class BinRecognizer
{
public:
    BinRecognizer(const YamlReader& cfg) : cfg_(cfg) {}
    boost::optional<vision::MsgFoundBin> find(const cv::Mat& frame, cv::Mat& out, Mode mode)
    {
        boost::optional<vision::MsgFoundBin> result;

        AllStripes white_stripes(cfg_.node("white"));
        auto wstripes = white_stripes.process(frame);

        AllStripes orange_stripes(cfg_.node("orange"));
        auto ostripes = orange_stripes.process(frame);

        vision::MsgFoundBin msg;
        for (auto& stripe : wstripes) {
            vision::MsgBin msg_bin;
            msg_bin.stripe = stripe.to_msg();
            msg_bin.locked = false;

            msg.bins.push_back(msg_bin);
        }

        for (auto& stripe : ostripes) {
            vision::MsgBin msg_bin;
            msg_bin.stripe = stripe.to_msg();
            msg_bin.locked = true;
            msg.bins.push_back(msg_bin);
        }

        return msg;
    }

private:
    YamlReader cfg_;
};

REGISTER_RECOGNIZER(BinRecognizer, bin);
