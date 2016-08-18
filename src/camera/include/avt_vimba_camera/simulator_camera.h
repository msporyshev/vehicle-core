#pragma once 

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>

#include <string>

namespace avt_vimba_camera {
class SimulatorCamera {
 public:
  SimulatorCamera(ros::NodeHandle nh, ros::NodeHandle nhp);
  ~SimulatorCamera(void);

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;

  std::string ip_;
  std::string guid_;
  std::string camera_info_url_;
  bool show_debug_prints_;

  bool get_from_storage_;
  std::string image_folder_;
  std::vector<std::string> image_storage_;

  image_transport::ImageTransport it_;
  // ROS Camera publisher
  image_transport::CameraPublisher pub_;
  ros::Timer timer_publish_;

  // sensor_msgs::CameraInfo left_info_;
  boost::shared_ptr<camera_info_manager::CameraInfoManager> info_man_;

  void publish_frame(const ros::TimerEvent& event);
  void create_sim_image(sensor_msgs::Image& output_image);
  void get_storaged_image(sensor_msgs::Image& output_image);
  void parse_image_storage();
};
}