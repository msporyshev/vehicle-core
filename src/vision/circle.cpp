#include <vision/MsgFoundCircle.h>
#include <config_reader/yaml_reader.h>
#include <point/point.h>

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <vector>
#include <string>

#include "common.h"
#include "rec_factory.h"
#include "image_pipeline.h"
#include "image_algorithm.h"

using namespace std;
using namespace cv;

class CircleRecognizer
{
public:
    CircleRecognizer(const YamlReader& cfg) : cfg_(cfg) {}

    boost::optional<vision::MsgFoundCircle> find(const cv::Mat& frame, cv::Mat& out, Mode mode)
    {
        boost::optional<vision::MsgFoundCircle> result;


        vector<Circle> all = find_by_color(frame, out, mode, "other");
        auto green = find_by_color(frame, out, mode, "green");


        copy(green.begin(), green.end(), back_inserter(all));

        if (all.empty()) {
            return result;
        }

        vision::MsgFoundCircle msg;
        for (auto& circle : all) {
            msg.circles.push_back(circle.to_msg());
        }

        return msg;
    }

    vector<Circle> find_by_color(const cv::Mat& frame, cv::Mat& out, Mode mode, string color_name)
    {
        ImagePipeline pipe(mode, color_name);
        pipe
            << HistEqualizer(cfg_)
            << BinarizerHSV(cfg_.node(color_name))
            << MedianFilter(cfg_.node("median"))
            ;

        auto bin = pipe.process(frame);
        // out = bin;

        vector<Circle> circles;

        int w = bin.cols;
        int h = bin.rows;
        Mat vis(h, w, CV_8U);
        memset(vis.data, 0, w*h);
        vector<int> qx(w * h);
        vector<int> qy(w * h);

        const int dx[] = {-1, 0, 1, 0};
        const int dy[] = {0, 1, 0, -1};

        for (int i = 0; i < h; i++)
            for (int j = 0; j < w; j++)
                //нашли очередную четырёхсвязную компоненту
                if (bin.at<uchar>(i, j) && !vis.at<uchar>(i, j)) {
                    int L = 0;
                    int R = 0;
                    qx[0] = i;
                    qy[0] = j;
                    vis.at<uchar>(i, j) = 1;
                    int imin = i, imax = i;
                    int jmin = j, jmax = j;
                    while (L <= R) {
                        for (int d = 0; d < 4; d++) {
                            int i1 = qx[L] + dx[d];
                            int j1 = qy[L] + dy[d];
                            if (i1 >= 0 && i1 < h && j1 >= 0 && j1 < w &&
                                bin.at<uchar>(i1, j1) && !vis.at<uchar>(i1, j1)) {
                                R++;
                                qx[R] = i1;
                                qy[R] = j1;
                                vis.at<uchar>(i1, j1) = 1;
                                imin = min(imin, i1); imax = max(imax, i1);
                                jmin = min(jmin, j1); jmax = max(jmax, j1);
                            }
                        }
                        L++;
                    }

                    //если компонента касается границы кадра - это плохо.
                    //Потому что шарик влезает не целиком
                    // if (jmin == 0 || jmin == w-1 || imin == 0 || imax == h-1)
                    //     continue;

                    // как соотносится площадь компоненты с площадью ограничивающего её прямоугольника?
                    int area_w = jmax - jmin + 1;
                    int area_h = imax - imin + 1;
                    // if (_w >= BALL_MINIMUM_DIAMETER && _h >= BALL_MINIMUM_DIAMETER
                        // && _w / (double)_h < BallRecognizer::BALL_BOUNDING_RECT_RATIO
                        // && _h / (double)_w < BallRecognizer::BALL_BOUNDING_RECT_RATIO
                        // && (R+1) / (double)(_w*_h) > BALL_SQDIF_PERCENT_LOW
                        // && (R+1) / (double)(_w*_h) < BALL_SQDIF_PERCENT_HIGH
                    // ) {
                    Circle circle(
                        Point((jmin + jmax)/2, (imin + imax)/2),
                        (1 + (jmax-jmin+imax-imin))/4
                    );

                    if (circle.r < min_radius_.get() || circle.r > max_radius_.get()) {
                        continue;
                    }

                    circles.push_back(circle);
                    // }
                }

        Color color = color_by_name.at(color_name);

        for (auto& circle : circles) {
            circle.draw(out, color, 2);
        }

        return circles;
    }

private:
    YamlReader cfg_;

    AUTOPARAM_OPTIONAL(double, min_radius_, 0);
    AUTOPARAM_OPTIONAL(double, max_radius_, 1e9);
};

REGISTER_RECOGNIZER(CircleRecognizer, circle);
