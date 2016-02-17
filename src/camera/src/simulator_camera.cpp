#include <avt_vimba_camera/simulator_camera.h>

#define DEBUG_PRINTS 1
#define DURATION_PERIOD 0.2

namespace avt_vimba_camera {

SimulatorCamera::SimulatorCamera(ros::NodeHandle nh, ros::NodeHandle nhp) : nh_(nh), nhp_(nhp), it_(nhp) {
  pub_  = it_.advertiseCamera("image_raw",  1);
  reconfigure_server_.setCallback(boost::bind(&avt_vimba_camera::SimulatorCamera::configure, this, _1, _2));
  timer_publish_ = nh_.createTimer(ros::Duration(DURATION_PERIOD), &SimulatorCamera::publish_frame, this);
}

SimulatorCamera::~SimulatorCamera(void) {
  pub_.shutdown();
}

void SimulatorCamera::create_sim_image(sensor_msgs::Image& output_image)
{
    static int color = 0;

    int height = ci_.height;
    int width = ci_.width;

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

void SimulatorCamera::publish_frame(const ros::TimerEvent& event)
{
    ros::Time ros_time = ros::Time::now();
    if (pub_.getNumSubscribers() > 0) {
        sensor_msgs::Image img;
        create_sim_image(img);
    
        ci_.header.stamp = img.header.stamp = ros_time;            
        img.header.frame_id = ci_.header.frame_id;
    
        pub_.publish(img, ci_);
    }
}

void SimulatorCamera::configure(Config& newconfig, uint32_t level) 
{
    if (newconfig.frame_id == "") {
      newconfig.frame_id = "camera";
    }
    updateCameraInfo(newconfig);
}

void SimulatorCamera::updateCameraInfo(const avt_vimba_camera::AvtVimbaCameraConfig& config) 
{
  // Set the frame id
  ci_.header.frame_id = config.frame_id;

  // Set the operational parameters in CameraInfo (binning, ROI)
  ci_.height    = config.height;
  ci_.width     = config.width;
  ci_.binning_x = config.binning_x;
  ci_.binning_y = config.binning_y;

  // ROI in CameraInfo is in unbinned coordinates, need to scale up
  ci_.roi.x_offset = config.roi_offset_x;
  ci_.roi.y_offset = config.roi_offset_y;
  ci_.roi.height   = config.roi_height;
  ci_.roi.width    = config.roi_width;
}

};
