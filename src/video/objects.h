#pragma once

#include "common.h"

#include <vector>

#include <opencv2/opencv.hpp>
#include <video/MsgStripe.h>
#include <point/point.h>

class Object
{
public:
    virtual void draw(cv::Mat& frame, Color color = Color::Orange, int thickness = 2) =0;
};

struct Contour: public Object
{
    std::vector<cv::Point> contour;

    Contour(const std::vector<cv::Point>& contour = std::vector<cv::Point>()): contour(contour) {}

    void draw(cv::Mat& frame, Color color, int thickness) override
    {
        for (int i = 1; i < contour.size(); i++) {
            cv::line(frame, contour[i - 1], contour[i], scalar_by_color.at(color), thickness);
        }
    }
};

using Segment = std::pair<cv::Point2d, cv::Point2d>;

struct Stripe: public Object
{

    Segment l, w;

    Stripe(Segment l = Segment(), Segment w = Segment()): l(l), w(w) {}

    void draw(cv::Mat& frame, Color color, int thickness) override
    {
        cv::line(frame, l.first, l.second, scalar_by_color.at(color), thickness);
        cv::line(frame, w.first, w.second, scalar_by_color.at(color), thickness);
    }

    video::MsgStripe to_msg() const
    {
        video::MsgStripe s;
        s.begin = MakePoint2(l.first.x, l.first.y);
        s.end = MakePoint2(l.second.x, l.second.y);
        s.wbegin = MakePoint2(w.first.x, w.first.y);
        s.wend = MakePoint2(w.second.x, w.second.y);
        s.width = width();
        return s;
    }

    double len() const
    {
        return norm(l.first - l.second);
    }

    double width() const
    {
        return norm(w.first - w.second);
    }
};

