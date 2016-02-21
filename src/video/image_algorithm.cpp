#include "image_algorithm.h"

#include <opencv2/opencv.hpp>
#include <vector>

#include <vector>

using namespace cv;
using namespace std;

namespace { // namespace

Mat equalize_channel(Mat src, double unused) {
    Mat res = src.clone();

    vector<double> hist(256);

    int all_count = src.rows * src.cols;

    int left_bound = 1000;
    int right_bound = 0;

    for (int i = 0; i < src.rows; i++) {
        for (int j = 0; j < src.cols; j++) {
            int val = src.at<uchar>(i, j);
            hist[val]+=1;
            right_bound = max(right_bound, val);
            left_bound = min(left_bound, val);
        }
    }

    int need_count = all_count * unused;
    int left_count = 0;
    int right_count = 0;
    int k = 0;
    while(left_count < need_count) {
        left_count += hist[k];
        k++;
    }
    left_bound = max(left_bound, k);
    k = hist.size() - 1;
    while(right_count < need_count) {
        right_count += hist[k];
        k--;
    }
    right_bound = min(right_bound, k);
    all_count -= left_count + right_count;

    if (right_bound == left_bound) {
        return res;
    }

    for (int i = 0; i < src.rows; i++) {
        for (int j = 0; j < src.cols; j++) {
            int oldval = src.at<uchar>(i, j);
            int newval = oldval;

            newval = 255.0 * (oldval - left_bound) / (right_bound - left_bound);
            newval = min(newval, 255);
            newval = max(newval, 0);
            res.at<uchar>(i, j) = newval;
        }
    }

    return res;
}


Stripe min_max_regression_segment(const std::vector<cv::Point> poly, double EPS)
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

} // namespace


cv::Mat BinarizerHSV::process(const cv::Mat& frame)
{
    Mat hsv;
    cvtColor(frame, hsv, CV_BGR2HSV);
    vector<Mat> channels(3);
    split(hsv, channels);

    Mat bin_h, bin_s, bin_v, bin_img;
    int hmin = h_min_.get();
    int hmax = h_max_.get();

    if (hmin >= 0) { //если отрицательное число, то это на самом деле оборот через 180 в H-канале
        inRange(channels[0], hmin, hmax, bin_h);
    } else {
        Mat range1, range2;
        inRange(channels[0], 180+hmin, 180, range1);
        inRange(channels[0], 0, hmax, range2);
        bitwise_or(range1, range2, bin_h);
    }

    int smin = s_min_.get();
    int smax = s_max_.get();

    int vmin = v_min_.get();
    int vmax = v_max_.get();

    inRange(channels[1], smin, smax, bin_s);
    inRange(channels[2], vmin, vmax, bin_v);

    bitwise_and(bin_s, bin_h, bin_img);
    bitwise_and(bin_v, bin_img, bin_img);

    return bin_img;
}

cv::Mat HistEqualizer::process(const cv::Mat& frame)
{
    vector<Mat> channels, channels_eq;
    split(frame, channels);

    channels_eq = channels;

    int channels_count = channels_count_.is_set() ? channels_count_.get() : channels.size();

    for (int i = 0; i < channels_count; i++) {
        channels_eq[i] = equalize_channel(channels[i], unused_.get());
    }

    Mat equalized;
    merge(channels_eq, equalized);

    return equalized;
}

cv::Mat GrayScale::process(const cv::Mat& frame)
{
    Mat result;
    cvtColor(frame, result, CV_BGR2GRAY);
    return result;
}


cv::Mat MedianFilter::process(const cv::Mat& frame)
{
    Mat result;
    medianBlur(frame, result, ksize_.get());
    return result;
}

cv::Mat GaussianFilter::process(const cv::Mat& frame)
{
    Mat result;
    GaussianBlur(frame, result, Size(kx_.get(), ky_.get()),
        sigma_x_.get(), sigma_y_.get());
    return result;
}

cv::Mat AbsDiffFilter::process(const cv::Mat& frame)
{
    Mat result;
    cv::absdiff(source_, frame, result);
    return result;
}

cv::Mat SobelFilter::process(const cv::Mat& frame)
{
    Mat result;
    Sobel(frame, result, ddepth_.get(), dx_.get(), dy_.get(), ksize_.get());
    return result;
}

cv::Mat LaplacianFilter::process(const cv::Mat& frame)
{
    Mat result;
    Laplacian(frame, result, -1, ksize_.get());
    return result;
}

cv::Mat FrameDrawer::process(const cv::Mat& frame)
{
    Mat result = frame.clone();

    int color = color_.get();
    int width = width_.get();

    for (int i = 0; i < result.rows; i++) {
        for (int j = 0; j < width; j++) {
            result.at<uchar>(i, j) = color;
            result.at<uchar>(i, result.cols - j - 1) = color;
        }
    }

    for (int i = 0; i < result.cols; i++) {
        for (int j = 0; j < width; j++) {
            result.at<uchar>(j, i) = color;
            result.at<uchar>(result.rows - j - 1, i) = color;
        }
    }

    return result;
}

cv::Mat ObjectDrawer::process(const cv::Mat& frame)
{
    Mat result = frame.clone();

    int width = width_.get();
    std::string color_name = color_.get();

    for (const auto& obj : objects_) {
        for (size_t i = 1; i < obj.size(); ++i) {
            cv::line(result, obj[i - 1], obj[i], scalar_by_color.at(color_by_name.at(color_name)),
                width);
        }
    }

    return result;
}

static std::vector<Contour> convert(const std::vector<std::vector<cv::Point>>& contours)
{
    std::vector<Contour> result;
    for (auto& contour : contours) {
        result.emplace_back(contour);
    }

    return result;
}

vector<Contour> FindContours::process(const Mat& image)
{
    std::vector<std::vector<cv::Point>> contours, approxes;
    std::vector<cv::Vec4i> hierarchy;

    findContours(image, contours, hierarchy,
        CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

    if (!approx_dist_.is_set()) {
        return convert(contours);
    }

    for (size_t i = 0; i < contours.size(); i++) {
        std::vector<cv::Point> approx;
        approxPolyDP(contours[i], approx, approx_dist_.get(), false);
        if (approx.size() < min_approx_count_.get() || approx.size() > max_approx_count_.get()) {
            continue;
        }

        approxes.push_back(approx);
    }

    return convert(approxes);
}

std::vector<Stripe> MinMaxStripes::process(const std::vector<Contour>& contours)
{
    std::vector<Stripe> result;
    for (auto& contour : contours) {
        result.push_back(min_max_regression_segment(contour.contour, 1e-4));
    }

    return result;
}
