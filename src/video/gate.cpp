#include "gate.h"
#include "image_pipeline.h"
#include "image_algorithm.h"
#include "rec_factory.h"

using namespace video;

boost::optional<MsgFoundGate> GateRecognizer::find(const cv::Mat& frame, cv::Mat& out, Mode mode)
{
    std::vector<Stripe> red_leg, green_leg;

    StripeRecognizer red_stripe(cfg_.node("red_stripe"));
    StripeRecognizer green_stripe(cfg_.node("green_stripe"));

    boost::optional<MsgFoundStripe> m;
    cv::Mat red_out = out.clone();
    cv::Mat green_out = out.clone();
    std::vector<Stripe> red_legs = to_stripe(*red_stripe.find(frame, red_out, mode));
    std::vector<Stripe> green_legs = to_stripe(*green_stripe.find(frame, green_out, mode));

    red_leg = take_leg(red_legs);
    green_leg = take_leg(green_legs);

    draw_gate(out, red_leg, green_leg);
    if (red_leg.empty() || green_leg.empty()) {
        return MsgFoundGate();
    } else {
        return msg(red_leg, green_leg);
    }
}

std::vector<Stripe> GateRecognizer::to_stripe(const MsgFoundStripe& msg)
{
    std::vector<Stripe> result(msg.stripes.size());
    size_t i(0);
    for (const auto& stripe : msg.stripes) {
        Segment line(std::make_pair(cv::Point2d(stripe.begin.x, stripe.begin.y),
            cv::Point2d(stripe.end.x, stripe.end.y)));
        Segment width(std::make_pair(cv::Point2d(stripe.wbegin.x, stripe.wbegin.y),
            cv::Point2d(stripe.wend.x, stripe.wend.y)));
        result[i++] = Stripe(line, width);
    }
    return result;
}

std::vector<Stripe> GateRecognizer::take_leg(std::vector<Stripe>& legs)
{
    std::sort(legs.begin(), legs.end(), [](Stripe first, Stripe second) {
        return first.len() > second.len();
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
        line(img, red_leg.begin()->l.first, red_leg.begin()->l.second, scalar_by_color.at(Color::Red), 2);
        line(img, red_leg.begin()->w.first, red_leg.begin()->w.second, scalar_by_color.at(Color::Red), 2);
    }
    if (!green_leg.empty()) {
        line(img, green_leg.begin()->l.first, green_leg.begin()->l.second, scalar_by_color.at(Color::Green), 2);
        line(img, green_leg.begin()->w.first, green_leg.begin()->w.second, scalar_by_color.at(Color::Green), 2);
    }
}

MsgFoundGate GateRecognizer::msg(const std::vector<Stripe>& red_leg, const std::vector<Stripe>& green_leg)
{
    std::vector<cv::Point> p = {red_leg.begin()->l.first, red_leg.begin()->l.second, green_leg.begin()->l.first, green_leg.begin()->l.second};
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
    m.gate.emplace_back();
    m.gate.front().left = red_leg.front().to_msg();
    m.gate.front().right = green_leg.front().to_msg();

    return m;
}

REGISTER_RECOGNIZER(GateRecognizer, gate);
