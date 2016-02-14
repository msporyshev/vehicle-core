#pragma once 

#include <avt_vimba_camera/avt_vimba_camera.h>
#include <avt_vimba_camera/AvtVimbaCameraConfig.h>
#include <avt_vimba_camera/avt_vimba_api.h>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>
#include <dynamic_reconfigure/server.h>

#include <string>

namespace avt_vimba_camera {
class SimulatorCamera {
 public:
  SimulatorCamera(ros::NodeHandle nh, ros::NodeHandle nhp);
  ~SimulatorCamera(void);

 private:
  AvtVimbaApi api_;
  AvtVimbaCamera cam_;

  diagnostic_updater::Updater updater_;
  diagnostic_updater::TopicDiagnostic* pub_freq_;

  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;

  std::string ip_;
  std::string guid_;
  std::string camera_info_url_;
  bool show_debug_prints_;

  image_transport::ImageTransport it_;
  // ROS Camera publisher
  image_transport::CameraPublisher pub_;
  sensor_msgs::CameraInfo ci_;
  ros::Timer timer_publish_;


  sensor_msgs::CameraInfo left_info_;
  boost::shared_ptr<camera_info_manager::CameraInfoManager> info_man_;

  // Dynamic reconfigure
  typedef avt_vimba_camera::AvtVimbaCameraConfig Config;
  typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
  ReconfigureServer reconfigure_server_;

  // Camera configuration
  Config camera_config_;

  void frameCallback(const FramePtr& vimba_frame_ptr);
  void configure(Config& newconfig, uint32_t level);
  void updateCameraInfo(const Config& config);
  void publish_frame(const ros::TimerEvent& event);
  void create_sim_image(sensor_msgs::Image& output_image);
};
}