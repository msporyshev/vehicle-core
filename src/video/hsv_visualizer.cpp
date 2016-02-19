#include <opencv2/opencv.hpp>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include <ros/ros.h>
#include <ros/package.h>

#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <ctime>

#include <config_reader/yaml_reader.h>

#include "image_pipeline.h"
#include "image_algorithm.h"

using namespace std;
using namespace cv;

namespace po = boost::program_options;
namespace fs = boost::filesystem;

struct {
    bool with_color_correction = false;
    string image_file;
    string features_file;

    void print(ostream& out) {
        out << "image file: " << image_file << endl
            << "features file: " << features_file << endl
            << "color correction: " << (with_color_correction ? "on" : "off");
    }
} params;


class InteractiveGrabcut {
public:
    cv::Mat src;
    cv::Mat fgd, bgd;
    bool ldrag = false;
    bool rdrag = false;
    bool ctrl_pressed = false;
    std::string name;

    cv::Mat mask;
    cv::Mat hsv;
    cv::Point lstart, rstart;
    cv::Scalar fg_color, bg_color;

    InteractiveGrabcut(const cv::Mat& src_)
            : name("igc")
            , fg_color(0, 0, 255), bg_color(255, 0, 0)
    {
        src = src_.clone();
        imshow("before correction", src_);

        YamlReader cfg("hsv_visualizer.yml", "video");
        if (params.with_color_correction) {
            ImagePipeline pipe;
            pipe << HistEqualizer(cfg);

            src = pipe.process(src);
            imshow("after correction", src);
        }

        mask = cv::Mat::ones(src.size(), CV_8U) * cv::GC_PR_BGD;
        cvtColor(src, hsv, CV_BGR2HSV);
        // cvtColor(src, hsv, CV_BGR2Lab);
        // hsv = src_.clone();
    }

    void prepare_window(const std::string name){
        this->name = name;
        cv::namedWindow(name);
        cv::setMouseCallback(name, events, this);
        // cv::createButton("plot", button_callback, this);
    }

    static void events( int e, int x, int y, int flags, void* s ){
        InteractiveGrabcut* self = (InteractiveGrabcut*)s;
        cv::Point pt(x, y);
        switch(e) {
        case CV_EVENT_LBUTTONDOWN:
            if (self->ctrl_pressed) {
                self->rdrag = true;
                self->rstart = pt;
                break;
            }
            self->ldrag = true;
            self->lstart = pt;
            break;
        case CV_EVENT_LBUTTONUP:
            if (self->ctrl_pressed) {
                self->rdrag = false;
                break;
            }
            self->ldrag = false;
            break;
        case CV_EVENT_RBUTTONDOWN:
            self->rdrag = true;
            self->rstart = pt;
            break;
        case CV_EVENT_RBUTTONUP:
            self->rdrag = false;
            break;
        case CV_EVENT_MOUSEMOVE:
            if(self->ldrag) {
                cv::line(self->mask, self->lstart, pt, cv::Scalar(cv::GC_FGD), 1);
                self->lstart = pt;
            }
            else if(self->rdrag) {
                cv::line(self->mask, self->rstart, pt, cv::Scalar(cv::GC_BGD), 1);
                self->rstart = pt;
            }
            break;
        default:
            break;
        };
    }

    void show(){
        cv::Mat scribbled_src = src.clone();
        const float alpha = 0.7f;
        for(int y=0; y < mask.rows; y++){
            for(int x=0; x < mask.cols; x++){
                if(mask.at<uchar>(y, x) == cv::GC_FGD) {
                    cv::circle(scribbled_src, cv::Point(x, y), 2, fg_color, -1);
                } else if(mask.at<uchar>(y, x) == cv::GC_BGD) {
                    cv::circle(scribbled_src, cv::Point(x, y), 2, bg_color, -1);
                } else if(mask.at<uchar>(y, x) == cv::GC_PR_BGD) {
                    cv::Vec3b& pix = scribbled_src.at<cv::Vec3b>(y, x);
                    pix[0] = (uchar)(pix[0] * alpha + bg_color[0] * (1-alpha));
                    pix[1] = (uchar)(pix[1] * alpha + bg_color[1] * (1-alpha));
                    pix[2] = (uchar)(pix[2] * alpha + bg_color[2] * (1-alpha));
                } else if(mask.at<uchar>(y, x) == cv::GC_PR_FGD) {
                    cv::Vec3b& pix = scribbled_src.at<cv::Vec3b>(y, x);
                    pix[0] = (uchar)(pix[0] * alpha + fg_color[0] * (1-alpha));
                    pix[1] = (uchar)(pix[1] * alpha + fg_color[1] * (1-alpha));
                    pix[2] = (uchar)(pix[2] * alpha + fg_color[2] * (1-alpha));
                }
            }
        }

        cv::imshow(name, scribbled_src);
        cv::imshow(name + "_FG", get_fg());
        cv::imshow("source", src);
    }

    void execute(){
        std::cout << "computing...";
        cv::grabCut(src, mask, cv::Rect(), bgd, fgd, 1, cv::GC_INIT_WITH_MASK);
        std::cout << "end." << std::endl;
    }

    static void button_callback(int state, void* self_) {
        InteractiveGrabcut* self = (InteractiveGrabcut*)self_;

        self->save_features();
        self->run_plotter();
    }

    void save_features() {
        ofstream output(params.features_file);

        output << "H,S,V,class" << endl;
        auto fg = get_fg();

        for (int i = 0; i < fg.rows; i++) {
            for (int j = 0; j < fg.cols; j++) {
                auto val = fg.at<Vec3b>(i, j);
                auto channels = hsv.at<Vec3b>(i, j);
                if (val[0] > 0 || val[1] > 0 || val[2] > 0) {
                    output << (int)channels[0] << "," << (int)channels[1] << "," << (int)channels[2] << "," << 1 << endl;
                } else {
                    output << (int)channels[0] << "," << (int)channels[1] << "," << (int)channels[2]  << "," << 0 << endl;
                }
            }
        }
    }

    void run_plotter() {
        std::string base_dir = ros::package::getPath("video");

        system(("python " + base_dir + "/visualizer.py " +
            params.features_file).c_str());
    }

    cv::Mat get_binmask(){
        cv::Mat binmask(mask.size(), CV_8U);
        binmask = mask & 1;
        return binmask;
    }

    cv::Mat get_fg(){
        cv::Mat fg = cv::Mat::zeros(src.size(), src.type());
        src.copyTo(fg, get_binmask());
        return fg;
    };

    template <class Func>
    void main_loop(Func func){
        while(1){
            int key = cv::waitKey(1);
            if(key == '\x1b')
                break;
            func(this, key);
            show();
        }
    }
};

void program_options_init(int argc, char** argv) {
    po::options_description desc("Usage");
    desc.add_options()
        ("help,h", "Produce help message")
        ("image,i", po::value(&params.image_file),
            "Path to image file")
        ("features-output,f", po::value(&params.features_file),
            "Specify <file path>.csv for hue and saturation values to be saved.\n"
            "Default is current folder and file name is the same as image file name")
        ("color-correction,c", po::value(&params.with_color_correction)->zero_tokens(),
            "Boolean flag. If specified, color correction is applied to source image.\n"
            "Color correction is about to increase red channel")
    ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (params.features_file.empty()) {
        fs::path features_path(params.image_file);
        params.features_file = features_path.stem().string() + ".csv";
    }


    if (vm.count("help") || params.image_file.empty()) {
        cout << desc << endl;
        exit(EXIT_SUCCESS);
    }

    params.print(cout);

}

int main(int argc, char** argv){
    program_options_init(argc, argv);

    cv::Mat image = cv::imread(params.image_file);

    InteractiveGrabcut igc(image);
    igc.prepare_window("target");
    igc.main_loop([](InteractiveGrabcut* self, int key)->void {
        // cout << "key: " << key << endl;
        if (key == 'a') {
            self->ctrl_pressed = !self->ctrl_pressed;
        } else if(key == ' ') {
            self->execute();

        } else if (key == '\r' || key == '\n') {
            self->show();
            waitKey(30);
            self->save_features();
            self->run_plotter();
        }
    });
}
