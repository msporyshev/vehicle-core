#include "stripe.h"
#include "image_pipeline.h"
#include "image_algorithm.h"
#include "rec_factory.h"

#include <algorithm>

#include <libauv/point/point.h>

using namespace video;

boost::optional<MsgFoundStripe> StripeRecognizer::find(const cv::Mat& frame, cv::Mat& out, Mode mode)
{
    ImagePipeline processor(mode);
    if (enable_col_cor_.is_set()) {
        processor << HistEqualizer(cfg_.node("equalizer"));
    }
    processor << BinarizerHSV(cfg_.node("binarizer"))
        << FrameDrawer(cfg_)
        << MedianFilter(cfg_.node("median_blur")) ;

    std::vector<Stripe> stripes = find_stripe(processor.process(frame));
    draw_stripe(out, stripes);
    return msg(stripes);
}

void StripeRecognizer::draw_stripe(cv::Mat& img, const std::vector<Stripe>& stripes)
{
    for (auto stripe : stripes) {
        line(img, stripe.l.first, stripe.l.second, scalar_by_color.at(Color::Orange), 2);
        line(img, stripe.w.first, stripe.w.second, scalar_by_color.at(Color::Orange), 2);
    }
}

MsgFoundStripe StripeRecognizer::msg(const std::vector<Stripe>& stripes)
{
    MsgFoundStripe m;
    int stripes_count = stripes.size();
    for (const auto& stripe : stripes) {
        m.stripes.push_back(stripe.to_msg());
    }

    return m;
}

std::vector<Stripe> StripeRecognizer::find_stripe(const cv::Mat& img)
{
    std::vector<Stripe> raw_stripes, stripes;
    std::vector<Stripe> result;

    raw_stripes = find_stripe_on_bin_img(img);

    cv::Scalar orange_color(scalar_by_color.at(Color::Orange));

    for (auto stripe : raw_stripes) {
        double side_ratio = stripe.len() / norm(stripe.w.first - stripe.w.second);
        if (side_ratio > sides_ratio_.get()) {
            stripes.push_back(stripe);
        }
    }

    return stripes;
}

std::vector<Stripe> StripeRecognizer::find_stripe_on_bin_img(const cv::Mat& img)
{
    bool use_min_width = min_stripe_width_.is_set();
    bool use_max_width = max_stripe_width_.is_set();

    bool use_min_length = min_stripe_length_.is_set();
    bool use_max_length = max_stripe_length_.is_set();

    bool use_max_approx_count = max_approx_count_.is_set();

    std::vector<std::vector<cv::Point> > contours, approxes;
    std::vector<cv::Vec4i> hierarchy;

    findContours(img, contours, hierarchy,
        CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

    for (size_t i = 0; i < contours.size(); i++) {
        std::vector<cv::Point> approx;
        approxPolyDP(contours[i], approx, approx_diff_.get(), false);
        if (use_max_approx_count && approx.size() > max_approx_count_.get()) {
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
        double length = norm(stripe.l.first - stripe.l.second);
        double width = norm(stripe.w.first - stripe.w.second);
        if (use_max_length && length > max_stripe_length_.get()) {
            continue;
        }

        if (use_min_length && length < min_stripe_length_.get()) {
            continue;
        }

        if (use_max_width && width > max_stripe_width_.get()) {
            continue;
        }

        if (use_min_width && width < min_stripe_width_.get()) {
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

