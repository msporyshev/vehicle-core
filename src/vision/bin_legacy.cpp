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

using namespace std;

class BinRecognizer
{
public:
    BinRecognizer(const YamlReader& cfg) : cfg_(cfg) {}
    boost::optional<vision::MsgFoundBin> find(const cv::Mat& frame, cv::Mat& out, Mode mode)
    {
        boost::optional<vision::MsgFoundBin> result;

        ImagePipeline processor(mode);

        processor
            << HistEqualizer(cfg_.node("equalizer"))
            << MedianFilter(cfg_.node("median"));

        Mat img = original_frame, bin, gray;

        vector<Point> quadrangles;
        vector<vector<Point> > scale_low, scale_high, buckets, diff_buckets;

        //фильнация изображения
        if (debug) {
            imshow("blur", img);
        }

        //поиск линий
        cvtColor(img, gray, CV_BGR2GRAY);
        Canny(gray, bin, CANNY_TH1, CANNY_TH2);
        binary_smooth(bin, bin, Size(2,2));
        if (debug) {
            imshow("quadrangles binarize", bin);
        }

        //поиск прямоугольника с изображением
        Mat quadrangles_frame;
        debug_frame.copyTo(quadrangles_frame);

        find_quadrangles_on_binarized_image(bin, ORANGE, quadrangles_frame, quadrangles, debug, true, true, false, cfg);
        vector<Point> p;
        for(size_t i = 0; i < quadrangles.size(); i += 4) {
            p.clear();
            for(size_t j = i; j < i + 4; j++) {
                p.push_back(quadrangles[j]);
            }
            buckets.push_back(p);
        }
        cout << "buckets size: " << buckets.size() << endl;
        //фильтрация найденных прямоугольников
        vector<int> wasted;
        remove_same_contours(buckets, wasted, CONTOURS_ACCURACY);

        cout << "wasted size: " << wasted.size() << endl;
        for(size_t i = 0; i < wasted.size(); i++)
            cout << wasted[i] << endl;

        for(size_t i = 0; i < wasted.size(); i++) {
            p.clear();
            for(int j = 0; j < 4; j++) {
                p.push_back(buckets[i][j]);
            }
            if (!wasted[i])
                diff_buckets.push_back(p);
        }

        for(size_t i = 0 ; i < buckets.size(); i++) {
            cout << "countour #" << i << endl;
            for (int j = 0; j < 4; j++) {
                cout << buckets[i][j] << ", ";
            }
            cout << endl;
        }

        cout << "diff_buckets size: " << diff_buckets.size() << endl;

        for(size_t i = 0; i < diff_buckets.size(); i++) {
            scale_high.push_back(scaling(diff_buckets[i], SCALING_HIGH_RATIO));
            scale_low.push_back(scaling(diff_buckets[i], SCALING_LOW_RATIO));

        }

        //для распознавания закрытой корзины
        Mat hsv_image;
        cvtColor(original_frame, hsv_image, CV_BGR2HSV);
        vector<Mat> hsv_channels;
        split(hsv_image, hsv_channels);
        Mat satur_chanel = hsv_channels[1];
        if(debug) {
            imshow("Finding locked bucket: Saturation channel", satur_chanel);
        }

        cvtColor(original_frame, gray, CV_BGR2GRAY);
        for(size_t i = 0; i < diff_buckets.size(); i++) {
            if(color_compare(gray, scale_low[i], scale_high[i], COLOR_DIFFERENCE)) {
                res.push_back(Bucket(diff_buckets[i]));
                res[res.size() - 1].locked = false;
                for (int j = 0; j < 4; j++) {
                    line(debug_frame, diff_buckets[i][j], diff_buckets[i][(j+1) % 4],
                        Scalar(DEBUG_B[GREEN], DEBUG_G[GREEN], DEBUG_R[GREEN]), DEBUG_LINE_WIDTH);
                }
            }
            LOG << "For finding locked bucket:" << endl;
            if(color_compare(satur_chanel, scale_high[i], scale_low[i], COLOR_DIFF_LOCKED)) {
                res.push_back(Bucket(diff_buckets[i]));
                res[res.size() - 1].locked = true;
                LOG << "this bucket is locked" << endl;
                for (int j = 0; j < 4; j++) {
                    line(debug_frame, diff_buckets[i][j], diff_buckets[i][(j+1) % 4],
                        Scalar(DEBUG_B[BLUE], DEBUG_G[BLUE], DEBUG_R[BLUE]), DEBUG_LINE_WIDTH);
                }
            }
        }

        cout << "res size: " << res.size() << endl;

        int silhouette_numer = -1;

        cout << "try to find sihluette...";
        for(size_t i =  0; i < res.size(); i++){
            if(res[i].locked) return;

            string str;
            stringstream ss;
            cout << "yes" << endl;
            silhouette_numer = classify_silhouette(res[i]);
            cout << "silhouette numer = " << silhouette_numer << endl;
            res[i].number = silhouette_numer;

            if(silhouette_numer != -1) {
                ss << silhouette_names[silhouette_numer];
                cout << "recognize " << silhouette_names[silhouette_numer] << endl;
            } else {
                ss << "NONE";
            }
            ss >> str;
            putText(debug_frame, str, Point(100,((i+1)*50)),
                    CV_FONT_HERSHEY_COMPLEX, 1, Scalar(0,0,255), 2);

    }

private:
    YamlReader cfg_;
};

REGISTER_RECOGNIZER(BinRecognizer, bin);
