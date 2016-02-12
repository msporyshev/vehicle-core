#include <iostream>
#include <vector>
#include <string>
#include <functional>
#include <utility>
#include <memory>

#include <video/MsgVideoFrame.h>
#include <video/MsgFoundBin.h>
#include <video/CmdSwitchCamera.h>

#include <camera/MsgCameraFrame.h>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include <libipc/ipc.h>
#include <yaml_reader.h>

#include "image_pipeline.h"
#include "common.h"
#include "image_algorithm.h"
#include "recognizer.h"
#include "rec_factory.h"

using namespace std;
using namespace video;
using namespace camera;

namespace po = boost::program_options;
namespace fs = boost::filesystem;

namespace { // namespace

struct CameraFrame
{
    int frameno = 0;
    Camera camera_type = Camera::None;
    Mode mode = Mode::Debug;
    cv::Mat mat;
    vector<pair<string, shared_ptr<RecognizerBase> > > recognizers;
} current_frame;


struct VideoParams
{
    string hostname = "localhost";
    string nodename = "video";
    string output_dir = ".";
    string singletest_file;
    string multitest_dir;
    vector<string> recognizer_names;
    size_t threads = 0;
    bool singletest = false;
    bool multitest = false;
    bool onboard = false;
    bool frames_from_cam = false;
    bool without_color_correction = false;
    bool use_stereo = false;

    string flag_to_str(bool flag) {
        return flag ? "on" : "off";
    }

    void print(ostream& out) {
        out << "hostname: " << hostname << endl
            << "ros node name: " << nodename << endl
            << "threadpool count: " << threads << endl
            << "recognizers: ";

        for (auto name : recognizer_names) {
            out << name << " ";
        }
        out << endl;

        out << "on board mode: " << flag_to_str(onboard) << endl;
        out << "recieving frames from camera device: " << flag_to_str(frames_from_cam) << endl;
        out << "color correction: " << flag_to_str(!without_color_correction) << endl;
        out << "on board mode: " << flag_to_str(onboard) << endl;
        out << "use stereo: " << flag_to_str(use_stereo) << endl;
    }

    string log_str() {
        stringstream ss;
        print(ss);
        return ss.str();
    }
} video_params;


template<typename T>
void load_mode_param(const po::variables_map& vm, string name, T& param, bool& flag)
{
    if (!vm.count(name)) {
        return;
    }

    param = vm[name].as<T>();
    flag = true;
}

void program_options_init(int argc, char** argv)
{
    po::options_description desc("Usage");
    desc.add_options()
        ("help,h", "Produce help message")
        ("onboard,o", po::value(&video_params.onboard)->zero_tokens(),
            "Boolean flag. If specified, all debug windows are disabled")
        ("frames-from-cam,f", po::value(&video_params.frames_from_cam)->zero_tokens(),
            "Boolean flag. If specified, video module takes frames from your camera device")
        ("dir,d", po::value(&video_params.output_dir), "Set output dir for saving frames, default=<current_dir>")
        ("threads,t", po::value(&video_params.threads), "Enable multithreaded mode, set threadpool threads count")
        ("singletest,s", po::value<string>(),
            "Set filename for single test mode. Enable singletest mode")
        ("multitest,m", po::value<string>(),
            "Set directory for multitest mode. Enable multitest mode")
        ("recognize,r", po::value(&video_params.recognizer_names)->multitoken(),
            // ("Set recognizers to work on input frames.\n" + possible_recognizers_str()).c_str())
            "")
        ("no-color-correction,n", po::value(&video_params.without_color_correction)->zero_tokens(),
            "If specified, video module doesn't produce color correction that is needed for underwater")
        ("ipc-host,i", po::value(&video_params.hostname), "Set ipc central ip address, default=localhost")
        ("nodename,a", po::value(&video_params.nodename), "Set this module name for ipc central, default=video")
        ("stereo-pair,p", po::value(&video_params.use_stereo)->zero_tokens(),
            "Boolean flag. If specified, video module takes frames in stereo mode")

    ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help")) {
        cout << desc << endl;
        exit(EXIT_SUCCESS);
    }

    load_mode_param(vm, "singletest", video_params.singletest_file, video_params.singletest);
    load_mode_param(vm, "multitest", video_params.multitest_dir, video_params.multitest);

    video_params.print(cout);
}

void rgb_to_gray(const cv::Mat& frame, cv::Mat& out)
{
    cv::cvtColor(frame, out, CV_BGR2GRAY);
}


void save_frame(const CameraFrame& frame_info, const cv::Mat& frame, string suffix)
{
    stringstream filename;
    filename << video_params.output_dir << "/";
    filename << frame_info.recognizers.front().first << "_"
        << camera_typename.at(frame_info.camera_type) << "_camera_"
        << setw(4) << setfill('0') << frame_info.frameno
        << suffix;

    imwrite(filename.str(), frame, {CV_IMWRITE_JPEG_QUALITY, 30});
}


cv::Mat process_frame(const CameraFrame& frame)
{
    cv::Mat result = frame.mat.clone();

    if (frame.mode == Mode::Debug) {
        cv::imshow("source frame", frame.mat);
    }

    for (auto& recognizer : frame.recognizers) {
        recognizer.second->process(frame.mat, result, frame.mode, frame.frameno, frame.camera_type);
    }

    if (frame.mode == Mode::Debug) {
        cv::imshow("result output", result);
        cv::waitKey();
    }

    return result;
}

Mode initial_mode()
{
    return video_params.onboard ? Mode::Onboard : Mode::Debug;
}

void run_single_test()
{
    CameraFrame frame;
    frame.mode = initial_mode();
    frame.mat = cv::imread(video_params.singletest_file);

    save_frame(frame, frame.mat, "in.png");
    auto res = process_frame(frame);
    save_frame(frame, res, "out.jpg");
}

void run_multitest()
{

}

void video_init(ipc::Communicator& comm)
{
}

} // namespace

int main(int argc, char** argv) {
    program_options_init(argc, argv);

    YamlReader cfg("video.yml", "video");

    if (video_params.singletest) {
        RegisteredRecognizers::instance().init_all(cfg, nullptr);

        run_single_test();
        return 0;
    }

    auto comm = make_shared<ipc::Communicator>(ipc::init(argc, argv, video_params.nodename));
    RegisteredRecognizers::instance().init_all(cfg, comm);

    ros::spin();
}