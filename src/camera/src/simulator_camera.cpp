#include <avt_vimba_camera/simulator_camera.h>

#define DEBUG_PRINTS 1
#define DURATION_PERIOD 0.2

namespace avt_vimba_camera {

SimulatorCamera::SimulatorCamera(ros::NodeHandle nh, ros::NodeHandle nhp) : nh_(nh), nhp_(nhp), it_(nhp), cam_("camera") {
  // Prepare node handle for the camera
  // TODO use nodelets with getMTNodeHandle()

  // Start Vimba & list all available cameras
  api_.start();

  // Set the image publisher before the streaming
  pub_  = it_.advertiseCamera("image_raw",  1);
  // Set the frame callback
  cam_.setCallback(boost::bind(&avt_vimba_camera::SimulatorCamera::frameCallback, this, _1));
  // Set the params
  // nhp_.param("ip", ip_, std::string(""));
  // nhp_.param("guid", guid_, std::string(""));
  // nhp_.param("camera_info_url", camera_info_url_, std::string(""));
  // std::string frame_id;
  // nhp_.param("frame_id", frame_id, std::string(""));
  // nhp_.param("show_debug_prints", show_debug_prints_, false);

  // Set camera info manager
  // info_man_  = boost::shared_ptr<camera_info_manager::CameraInfoManager>(new camera_info_manager::CameraInfoManager(nhp_, frame_id, camera_info_url_));

  // Start dynamic_reconfigure & run configure()
  reconfigure_server_.setCallback(boost::bind(&avt_vimba_camera::SimulatorCamera::configure, this, _1, _2));
  ROS_INFO_STREAM("Try to create");
  ros::Timer timer_publish_ = nh_.createTimer(ros::Duration(DURATION_PERIOD), &SimulatorCamera::publish_frame, this);
  ROS_INFO_STREAM("Exit");

}

void SimulatorCamera::frameCallback(const FramePtr& vimba_frame_ptr) {
  // ros::Time ros_time = ros::Time::now();
  // ROS_INFO("Frame callback entered");
  // if (pub_.getNumSubscribers() > 0) {
  //   sensor_msgs::Image img;
  //   if (api_.frameToImage(vimba_frame_ptr, img)) {
  //     sensor_msgs::CameraInfo ci = info_man_->getCameraInfo();
  //     ci.header.stamp = img.header.stamp = ros_time;
  //     img.header.frame_id = ci.header.frame_id;
  //     pub_.publish(img, ci);
  //   } else {
  //     ROS_WARN_STREAM("Function frameToImage returned 0. No image published.");
  //   }
  // }
  // updater_.update();
}

SimulatorCamera::~SimulatorCamera(void) {
  // cam_.stop();
  pub_.shutdown();
}

void SimulatorCamera::create_sim_image(sensor_msgs::Image& output_image)
{
    static int color = 0;
    color++;
    if(color == 255) {
        color = 0;
    }

    // unsigned char* image_pointer_red = (unsigned char*) malloc (sizeof(unsigned char) * width * height);
    // unsigned char* image_pointer_green = (unsigned char*) malloc (sizeof(unsigned char) * width * height);
    // unsigned char* image_pointer_blue = (unsigned char*) malloc (sizeof(unsigned char) * width * height);

    // uint8_t *uint8_pointer_red    = reinterpret_cast<uint8_t*>((unsigned char*)image_pointer_red[0].L());
    // uint8_t *uint8_pointer_green  = reinterpret_cast<uint8_t*>((unsigned char*)image_pointer_green[0].L());
    // uint8_t *uint8_pointer_blue   = reinterpret_cast<uint8_t*>((unsigned char*)image_pointer_blue[0].L());

    int height = ci_.height;
    int width = ci_.width;

    // sensor_msgs::Image output_image;
    output_image.header.stamp     = ros::Time::now();
    output_image.height           = height;
    output_image.width            = width;
    output_image.encoding         = "rgb8";
    output_image.is_bigendian     = false;
    output_image.step             = 3 * height;

    for(int i = 0; i < (width * height); i++)
    {
        output_image.data.push_back(color);
        output_image.data.push_back(color);
        output_image.data.push_back(color);

        // output_image.data.push_back(uint8_pointer_red[i]);
        // output_image.data.push_back(uint8_pointer_green[i]);
        // output_image.data.push_back(uint8_pointer_blue[i]);
    }
}

void SimulatorCamera::publish_frame(const ros::TimerEvent& event)
{
  ROS_INFO_STREAM("Try to publish");
    ros::Time ros_time = ros::Time::now();
    if (pub_.getNumSubscribers() > 0) {
        sensor_msgs::Image img;
        create_sim_image(img);
    
        // ci_.header.frame_id = ci_.header.frame_id + 1;
        ci_.header.stamp = img.header.stamp = ros_time;
            
        img.header.frame_id = ci_.header.frame_id;
    
        pub_.publish(img, ci_);
        ROS_INFO_STREAM("published frame");
    }
}

/** Dynamic reconfigure callback
*
*  Called immediately when callback first defined. Called again
*  when dynamic reconfigure starts or changes a parameter value.
*
*  @param newconfig new Config values
*  @param level bit-wise OR of reconfiguration levels for all
*               changed parameters (0xffffffff on initial call)
**/
void SimulatorCamera::configure(Config& newconfig, uint32_t level) 
{
  // try {
  //   // resolve frame ID using tf_prefix parameter
  //   if (newconfig.frame_id == "") {
  //     newconfig.frame_id = "camera";
  //   }
  //   // The camera already stops & starts acquisition
  //   // so there's no problem on changing any feature.
  //   if (!cam_.isOpened()) {
  //     cam_.start(ip_, guid_, show_debug_prints_);
  //   }
  //   cam_.updateConfig(newconfig);
    updateCameraInfo(newconfig);
  // } catch (const std::exception& e) {
  //   ROS_ERROR_STREAM("Error reconfiguring mono_camera node : " << e.what());
  // }
}

void SimulatorCamera::updateCameraInfo(const avt_vimba_camera::AvtVimbaCameraConfig& config) 
{

  // Get camera_info from the manager
  // sensor_msgs::CameraInfo ci = info_man_->getCameraInfo();

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

  // // set the new URL and load CameraInfo (if any) from it
  // std::string camera_info_url;
  // nhp_.getParam("camera_info_url", camera_info_url);
  // if (camera_info_url != camera_info_url_) {
  //   info_man_->setCameraName(config.frame_id);
  //   if (info_man_->validateURL(camera_info_url)) {
  //     info_man_->loadCameraInfo(camera_info_url);
  //     ci = info_man_->getCameraInfo();
  //   } else {
  //     ROS_WARN_STREAM("Camera info URL not valid: " << camera_info_url);
  //   }
  // }

  // bool roiMatchesCalibration = (ci.height == config.roi_height
  //                             && ci.width == config.roi_width);
  // bool resolutionMatchesCalibration = (ci.width == config.width
  //                                  && ci.height == config.height);
  // // check
  // ci.roi.do_rectify = roiMatchesCalibration || resolutionMatchesCalibration;

  // // push the changes to manager
  // info_man_->setCameraInfo(ci);
}

};
