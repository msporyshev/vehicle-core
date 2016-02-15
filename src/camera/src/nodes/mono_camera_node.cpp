#include <ros/ros.h>
#include <avt_vimba_camera/mono_camera.h>
#include <avt_vimba_camera/simulator_camera.h>

#include <boost/program_options.hpp>
namespace po = boost::program_options;

bool program_options_init(int argc, char* argv[])
{
    po::options_description desc("Usage");
    desc.add_options()
        ("help,h", "Produce help message.")
        ("simulating,s", po::bool_switch()->default_value(false), "Enable simulating mode");

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    if (vm.count("help")) {
        std::cout << desc << std::endl;
        exit(EXIT_SUCCESS);
    }

    return vm["simulating"].as<bool>();
}


int main(int argc, char** argv)
{
    int is_simulating = program_options_init(argc, argv);
    
    ros::init(argc, argv, "mono_camera_node");

    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    avt_vimba_camera::MonoCamera* mc;
    avt_vimba_camera::SimulatorCamera* sc;
    if(!is_simulating) {
        ROS_INFO_STREAM("Set real mode");
        mc = new avt_vimba_camera::MonoCamera(nh,nhp);
    } else {
        ROS_INFO_STREAM("Set simulation mode");
        sc = new avt_vimba_camera::SimulatorCamera(nh,nhp);
    }
    ros::spin();
    return 0;
}