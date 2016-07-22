#pragma once

#include "common.h"

#include <vector>
#include <iostream>

#include <opencv2/opencv.hpp>
#include <vision/MsgStripe.h>
#include <vision/MsgCircle.h>
#include <point/point.h>



class Object
{
public:
    virtual void draw(cv::Mat& frame, Color color = Color::Orange, int thickness = 2) const =0;
};

struct Contour: public Object
{
    std::vector<cv::Point> contour;

    Contour(const std::vector<cv::Point>& contour = std::vector<cv::Point>()): contour(contour) {}

    void draw(cv::Mat& frame, Color color, int thickness) const override
    {
        for (int i = 1; i < contour.size(); i++) {
            cv::line(frame, contour[i - 1], contour[i], scalar_by_color.at(color), thickness);
        }
    }
};

struct Hist: public Object
{
    std::vector<int> count;

    Hist(const std::vector<int>& count = std::vector<int>()): count(count) {}

    void draw(cv::Mat& frame, Color color, int thickness = 0) const override {
        frame = cv::Mat::zeros(frame.rows, frame.cols, CV_8UC3);
        int bar_width = (frame.cols + count.size() - 1) / count.size();

        std::cout << frame.cols << " " << count.size() << std::endl;
        std::cout << bar_width << std::endl;

        std::vector<double> ps(count.size());
        std::copy(count.begin(), count.end(), ps.begin());



        double sum = 0;
        for (auto c : count) {
            sum += c;
        }

        double pmax = 0;
        for (auto& p : ps) {
            p /= sum;
            pmax  = std::max(pmax, p);
        }

        int x = 0;
        for (double p : ps) {
            int len = frame.rows * p / pmax;

            cv::line(frame,
                cv::Point(x, frame.rows), cv::Point(x, frame.rows - len),
                scalar_by_color.at(color), bar_width);

            x += bar_width;
        }
    }
};

struct Circle: public Object
{
    cv::Point center;
    double r;

    Circle(cv::Point center = cv::Point(), double r = 0): center(center), r(r) {}

    void draw(cv::Mat& frame, Color color, int thickness) const override
    {
        cv::circle(frame, center, 4, cv::Scalar(211, 0, 148), -1);
        cv::circle(frame, center, r, scalar_by_color.at(color), thickness);
    }

    vision::MsgCircle to_msg() const
    {
        vision::MsgCircle c;
        c.center = Point2d(center.x, center.y);
        c.radius = r;
        return c;
    }
};

using Segment = std::pair<cv::Point2d, cv::Point2d>;

struct Stripe: public Object
{
    Segment l, w;

    Stripe(Segment l = Segment(), Segment w = Segment()): l(l), w(w) {}

    void draw(cv::Mat& frame, Color color, int thickness) const override
    {
        cv::line(frame, l.first, l.second, scalar_by_color.at(color), thickness);
        cv::line(frame, w.first, w.second, scalar_by_color.at(color), thickness);
    }

    vision::MsgStripe to_msg() const
    {
        vision::MsgStripe s;
        s.begin = Point2d(l.first.x, l.first.y);
        s.end = Point2d(l.second.x, l.second.y);
        s.wbegin = Point2d(w.first.x, w.first.y);
        s.wend = Point2d(w.second.x, w.second.y);
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

