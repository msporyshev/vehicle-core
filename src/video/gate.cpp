#include "gate.h"
#include "image_pipeline.h"
#include "image_algorithm.h"
#include "rec_factory.h"

using namespace video;

MsgFoundGate GateRecognizer::find(const cv::Mat& frame, cv::Mat& out, Mode mode)
{
    std::vector<Stripe> red_leg, green_leg;

    ImagePipeline processor(mode);
    processor << BinarizerHSV(cfg_.node("red_binarizer"));

    StripeRecognizer stripe_rec(cfg_.node("stripe"));
    std::vector<Stripe> red_legs = stripe_rec.find_stripe(processor.process(frame));

    processor.clear_processors();
    processor << BinarizerHSV(cfg_.node("green_leg"));
    std::vector<Stripe> green_legs = stripe_rec.find_stripe(processor.process(frame));
    
    red_leg = take_leg(red_legs);
    green_leg = take_leg(green_legs);

    draw_gate(out, red_leg, green_leg);
    if (red_leg.empty() || green_leg.empty()) {
        return MsgFoundGate();
    } else {
        return msg(red_leg, green_leg);
    }
}

std::vector<Stripe> GateRecognizer::take_leg(std::vector<Stripe>& legs)
{
    std::sort(legs.begin(), legs.end(), [](Stripe first, Stripe second) {
        return first.length() > second.length();
    });

    std::vector<Stripe> result;
    for (auto iter = legs.begin(); iter != legs.end(); ++iter) {
        if (is_horizontal(*iter)) {
            continue;
        }
        result.push_back(*iter);
        break;
    }

    return result;
}

void GateRecognizer::draw_gate(cv::Mat& img, const std::vector<Stripe>& red_leg, const std::vector<Stripe>& green_leg)
{
    if (!red_leg.empty()) {
        line(img, red_leg.begin()->line.first, red_leg.begin()->line.second, scalar_by_color.at(Color::Red), 2);
        line(img, red_leg.begin()->width.first, red_leg.begin()->width.second, scalar_by_color.at(Color::Red), 2);
    }
    if (!green_leg.empty()) {
        line(img, green_leg.begin()->line.first, green_leg.begin()->line.second, scalar_by_color.at(Color::Green), 2);
        line(img, green_leg.begin()->width.first, green_leg.begin()->width.second, scalar_by_color.at(Color::Green), 2);
    }
}

MsgFoundGate GateRecognizer::msg(const std::vector<Stripe>& red_leg, const std::vector<Stripe>& green_leg)
{
    std::vector<cv::Point> p = {red_leg.begin()->line.first, red_leg.begin()->line.second, green_leg.begin()->line.first, green_leg.begin()->line.second};
    std::sort(p.begin(), p.end(), [](cv::Point a, cv::Point b) {
        return a.x < b.x;
    });

    if (p[0].y < p[1].y) {
        std::swap(p[0], p[1]);
    }
    if (p[3].y < p[2].y) {
        std::swap(p[2], p[3]);
    }

    MsgFoundGate m;
    for (int i = 0; i < p.size(); ++i) {
        m.gate.cols[i] = p[i].x;
        m.gate.rows[i] = p[i].y;
    }

    return m;
}

REGISTER_RECOGNIZER(GateRecognizer, gate);
