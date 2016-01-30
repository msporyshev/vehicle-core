#include "stripe.h"
#include "image_pipeline.h"
#include "image_algorithm.h"
#include "rec_factory.h"

using namespace video;

MsgFoundBin StripeRecognizer::find(const cv::Mat& frame, cv::Mat& out, Mode mode)
{
    ImagePipeline processor(mode);
    processor << BinarizerHSV(cfg_)
        << FrameDrawer(cfg_)
        << MedianBlur(cfg_);

    out = processor.process(frame);

    return MsgFoundBin();
}




