#include "stripe.h"
#include "image_pipeline.h"
#include "image_algorithm.h"
#include "rec_factory.h"

#include <algorithm>

using namespace video;

MsgFoundBin StripeRecognizer::find(const cv::Mat& frame, cv::Mat& out, Mode mode)
{
    ImagePipeline processor(mode);
    processor << BinarizerHSV(cfg_)
        << FrameDrawer(cfg_)
        << MedianBlur(cfg_);

    out = processor.process(frame);

    std::vector<cv::Point> stripes = find_stripe(out);

    size_t n = stripes.size() / 4;
    if (n > 2) n = 2;
    std::vector<cv::Point> tmp;
    for (size_t i = 0; i < n*4; i += 4) {
        for (int j = 0; j < 4; j++)
            tmp.push_back(stripes[i+j]);
    }

    return fill_msg(tmp);
}

MsgFoundBin StripeRecognizer::fill_msg(const std::vector<cv::Point>& stripes)
{
    MsgFoundBin m;
    int stripes_count = stripes.size() / 4;
    for (size_t i = 0; i < stripes_count; i += 4) {
        MsgBin bin;
        for (int j = i; j < i + 4; ++j) {
            bin.rows[j] = stripes[j].y;
            bin.cols[j] = stripes[j].x;
        }
        m.bins.push_back(bin);
    }

    return m;
} 

std::vector<cv::Point> StripeRecognizer::find_stripe(cv::Mat& img)
{
    std::vector<Stripe> raw_stripes, stripes;
    std::vector<cv::Point> result;

    raw_stripes = find_stripe_on_bin_img(img);

    //TODO: Вынести в константы
    cv::Scalar orange_color(30, 75, 250);

    cfg_.read_param(sides_ratio_, "sides_ratio");

    for (auto stripe : raw_stripes) {
        double side_ratio = stripe.length() / norm(stripe.width.first - stripe.width.second);
        if (side_ratio > sides_ratio_) {
            stripes.push_back(stripe);
            line(img, stripe.line.first, stripe.line.second, orange_color, 2, CV_AA, 0);
        }
    }

    for (const auto& stripe : stripes) {
        auto dir = (stripe.width.first - stripe.width.second) * 0.5;

        std::vector<cv::Point> tmp = {stripe.line.first - dir,
            stripe.line.first + dir,
            stripe.line.second + dir,
            stripe.line.second - dir};

        cv::Point center;
        for (auto point : tmp) {
            center += point;
        }

        center.x /= 4;
        center.y /= 4;

        result.insert(result.end(), tmp.begin(), tmp.end());
        circle(img, center, 3, cv::Scalar(0, 255, 0), -1);
        for (size_t i = 0; i < 4; i++) {
            line(img, tmp[i], tmp[(i+1) % 4], orange_color, 1, CV_AA, 0);
        }
    }

    return result;
}

std::vector<Stripe> StripeRecognizer::find_stripe_on_bin_img(cv::Mat& img)
{
    cv::Scalar color(30, 75,  250);

    double MIN_WIDTH, MAX_WIDTH, MIN_LENGTH, MAX_LENGTH, APPROX_DIFF;
    int MAX_APPROX_COUNT;

    bool use_min_width = cfg_.is_param_readable(MIN_WIDTH, "min_stripe_width");
    bool use_max_width = cfg_.is_param_readable(MAX_WIDTH, "max_stripe_width");

    bool use_min_length = cfg_.is_param_readable(MIN_LENGTH, "min_stripe_length");
    bool use_max_length = cfg_.is_param_readable(MAX_LENGTH, "max_stripe_length");

    cfg_.read_param(APPROX_DIFF, "approx_diff");
    bool use_max_approx_count = cfg_.is_param_readable(MAX_APPROX_COUNT, "max_approx_count");

    std::vector<std::vector<cv::Point> > contours, approxes;
    std::vector<cv::Vec4i> hierarchy;

    findContours(img, contours, hierarchy,
        CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

    for (size_t i = 0; i < contours.size(); i++) {
        std::vector<cv::Point> approx;
        approxPolyDP(contours[i], approx, APPROX_DIFF, false);
        if (use_max_approx_count && approx.size() > MAX_APPROX_COUNT) {
            continue;
        }

        approxes.push_back(approx);
    }

    std::vector<Stripe> stripes;
    for (int i = 0; i < approxes.size(); i++) {
        auto stripe = min_max_regression_segment(approxes[i]);

        stripes.push_back(stripe);
    }

    std::vector<Stripe> res;

    for (const auto& stripe : stripes) {
        double length = norm(stripe.line.first - stripe.line.second);
        double width = norm(stripe.width.first - stripe.width.second);

        if (use_max_length && length > MAX_LENGTH) {
            continue;
        }

        if (use_min_length && length < MIN_LENGTH) {
            continue;
        }

        if (use_max_width && width > MAX_WIDTH) {
            continue;
        }

        if (use_min_width && width < MIN_WIDTH) {
            continue;
        }

        res.push_back(stripe);
    }

    return res;
}

Stripe StripeRecognizer::min_max_regression_segment(const std::vector<cv::Point> poly, double EPS)
{
    const double INF = 1e20;

    std::vector<cv::Point> hull;

    convexHull(poly, hull);

    cv::Point2d bestn, d, begin, end;
    double minmaxd = INF, mid;
    std::vector<double> dists(hull.size());

    // ищем вектор, доставляющий максимум проекции на него выпуклой оболочки.
    for(size_t j = 0; j < hull.size(); j++) {
        cv::Point &s = hull[j], &f = hull[(j + 1) % hull.size()];
        double a = f.y - s.y,
            b = s.x - f.x;


        for(size_t k = 0; k < hull.size(); k++)
            dists[k] = (a * hull[k].x + b * hull[k].y) / sqrt(a * a + b * b);

        double maxd = -INF, mind = INF;

        for(size_t k = 0; k < dists.size(); k++) {
            maxd = std::max(maxd, dists[k]);
            mind = std::min(mind, dists[k]);
        }

        if (maxd - mind < minmaxd) {
            minmaxd = maxd - mind;
            mid = (maxd + mind) / 2;
            bestn.x = a;
            bestn.y = b;
        }
    }

    d = bestn * (mid / norm(bestn));
    cv::Point2d n(-bestn.y / norm(bestn), bestn.x / norm(bestn));
    double mint = INF, maxt = -INF;

    for (size_t j = 0; j < hull.size(); j++) {
        double t = n.x * hull[j].x + n.y * hull[j].y;
        mint = cv::min(mint, t);
        maxt = cv::max(maxt, t);
    }

    begin.x = n.x * mint + d.x;
    begin.y = n.y * mint + d.y;
    end.x = n.x * maxt + d.x;
    end.y = n.y * maxt + d.y;

    cv::Point2d w1, w2;
    double width = 0;

    for (size_t j = 0; j < poly.size(); j++) { // â ãðóïïû îòðåçêè äîáàâëÿëèñü ïàðàìè òî÷åê, òàê ÷òî èõ âñåãäà ÷åòíîå ÷èñëî
        const cv::Point &p0 = poly[j];

        cv::Point minpt, maxpt;
        for (size_t k = 0; k < poly.size(); k++) {
            const cv::Point &start = poly[k], b = start - p0;
            cv::Point curn(poly[(k + 1) % poly.size()] - start);

            double det = -bestn.x * curn.y * 1.0l + bestn.y * curn.x,
                dt1 = -b.x * curn.y + b.y * curn.x,
                dt2 = bestn.x * b.y - bestn.y * b.x,
                t2, t1;

            if (det == 0) continue;

            t2 = dt2 / det;
            t1 = dt1 / det;
            cv::Point2d point = curn * t2 + start;

            if (t2 >= -EPS && t2 <= 1 + EPS && fabsl(t1) > EPS) {
                minpt = p0;
                maxpt = point;
                break;
            }

        }

        double r = norm(minpt - maxpt);

        if (width < r) {
            w1 = minpt;
            w2 = maxpt;
            width = r;
        }
    }

    return Stripe(Segment(begin, end), Segment(w1, w2));
}
