#include <avt_vimba_camera/simulator_camera.h>

#include <boost/filesystem.hpp>
#include <opencv2/opencv.hpp>

#define DEBUG_PRINTS 1
#define PUBLISH_PERIOD 0.2

namespace fs = boost::filesystem;

namespace avt_vimba_camera {

SimulatorCamera::SimulatorCamera(ros::NodeHandle nh, ros::NodeHandle nhp) : nh_(nh), nhp_(nhp), it_(nhp) {
  float publish_period;
  
  pub_  = it_.advertiseCamera("image_raw",  1);
    // Set the params
  nhp_.param("ip", ip_, std::string(""));
  nhp_.param("guid", guid_, std::string(""));
  nhp_.param("camera_info_url", camera_info_url_, std::string(""));
  std::string frame_id;
  nhp_.param("frame_id", frame_id, std::string(""));
  nhp_.param("show_debug_prints", show_debug_prints_, false);
  nhp_.param("publish_period", publish_period, (float)PUBLISH_PERIOD);
  nhp_.param("get_from_storage", get_from_storage_, false);
  nhp_.param("image_folder", image_folder_, std::string(""));

  timer_publish_ = nh_.createTimer(ros::Duration(publish_period), &SimulatorCamera::publish_frame, this);

  if(get_from_storage_) {
    parse_image_storage();
  }

  // Set camera info manager
  info_man_  = boost::shared_ptr<camera_info_manager::CameraInfoManager>(new camera_info_manager::CameraInfoManager(nhp_, frame_id, camera_info_url_));
}

SimulatorCamera::~SimulatorCamera(void) {
  pub_.shutdown();
}

void SimulatorCamera::create_sim_image(sensor_msgs::Image& output_image)
{
    static int color = 0;

    sensor_msgs::CameraInfo ci = info_man_->getCameraInfo();

    int height = ci.height;
    int width = ci.width;

    output_image.header.stamp     = ros::Time::now();
    output_image.height           = height;
    output_image.width            = width;
    output_image.encoding         = "rgb8";
    output_image.is_bigendian     = false;
    output_image.step             = 3 * width;

    for(int i = 0; i < width; i++) {
      
      if(color == 255) {
        color = 0;
      } else {
        color++;  
      }

      for(int j = 0; j < height; j++) {
        output_image.data.push_back(color);
        output_image.data.push_back(255 - color);
        output_image.data.push_back(color);
      }
    }
}

void SimulatorCamera::parse_image_storage()
{
  if(image_folder_ == "") {
    return;
  }

  for (auto it = fs::directory_iterator(image_folder_); it != fs::directory_iterator(); ++it) {
    if (it->path().extension().string() == ".png") {
        image_storage_.push_back(it->path().string());
        ROS_WARN_STREAM("image found: " << it->path().string());
    }
  }
}

void SimulatorCamera::get_storaged_image(sensor_msgs::Image& output_image)
{
  if(image_storage_.size() == 0) {
    ROS_WARN_STREAM("image storage is empty");
    return;
  }

  static std::vector<std::string>::iterator image_iterator = image_storage_.begin();

  if(image_iterator == image_storage_.end() - 1) {
    image_iterator = image_storage_.begin();    
  } else {
    image_iterator++;
  }

  cv::Mat mat = cv::imread(*image_iterator);

  cv_bridge::CvImage bridge_msg;
  bridge_msg.encoding = "bgr8";
  bridge_msg.image = mat;
  output_image = *bridge_msg.toImageMsg();
}

void SimulatorCamera::publish_frame(const ros::TimerEvent& event)
{
    ros::Time ros_time = ros::Time::now();
    if (pub_.getNumSubscribers() > 0) {
        sensor_msgs::Image img;
        if(!get_from_storage_) {
          create_sim_image(img);
        } else {
          get_storaged_image(img);
        }
        
        sensor_msgs::CameraInfo ci = info_man_->getCameraInfo();
        
        cv::Mat mat = cv_bridge::toCvCopy(img, "bgr8")->image;
        cv::Size size(ci.width,ci.height);
        cv::resize(mat, mat, size);

        cv_bridge::CvImage bridge_msg;
        bridge_msg.encoding = "bgr8";
        bridge_msg.image = mat;
        img = *bridge_msg.toImageMsg();

        ci.header.stamp = img.header.stamp = ros_time;            
        img.header.frame_id = ci.header.frame_id;
    
        pub_.publish(img, ci);
    }
}
};
