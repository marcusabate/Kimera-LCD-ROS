/** LCD_ros
  *
  * @file LCD_ros.h
  * @brief ROS wrapper for VIO Loop Closure Detector and rosbags as datasource
  * @author Marcus Abate
  *
**/

#pragma once

#include <LoopClosureDetector.h>
#include <StereoFrame.h>
#include <common/vio_types.h>
#include <datasource/DataSource.h>
#include <VioFrontEndParams.h>

#include <opencv2/core/core.hpp>
#include <opencv2/core/matx.hpp>

#include "StereoBuffer.h"

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/console.h>
#include <ros/spinner.h>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <image_transport/image_transport.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/subscriber_filter.h>

#include <gtsam/geometry/Pose3.h>

typedef image_transport::SubscriberFilter ImageSubscriber;
typedef message_filters::sync_policies::ApproximateTime
    <sensor_msgs::Image, sensor_msgs::Image> sync_pol;

// TODO: need some kind of output for detections

namespace VIO {

struct StereoCalibration {
  CameraParams left_camera_info_;
  CameraParams right_camera_info_;
  gtsam::Pose3 camL_Pose_camR_; // relative pose between cameras
};

struct LCDTaggedFrame {
  Timestamp timestamp_;
  cv::Mat left_img_;
  geometry_msgs::TransformStamped transform_;
};

class LCD_ros {
public:
  LCD_ros();

  ~LCD_ros() {}

  bool spin();

private:
  cv::Mat readRosImage(const sensor_msgs::ImageConstPtr& img_msg);

  cv::Mat readRosRGBImage(const sensor_msgs::ImageConstPtr& img_msg);

  cv::Mat readRosDepthImage(const sensor_msgs::ImageConstPtr& img_msg);

  bool parseCameraData(StereoCalibration* stereo_calib);

  void callbackCamAndProcessStereo(const sensor_msgs::ImageConstPtr& msgLeft,
                                   const sensor_msgs::ImageConstPtr& msgRight);

  void leftCameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);

  void rightCameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);

  void publishOutput(const LoopClosureDetectorOutputPayload& payload);

  void print() const;

private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;
  ros::Publisher lcd_info_pub_;
  ros::Publisher lcd_marker_pub_;
  ros::Publisher lcd_pose_array_pub_;
  ros::Publisher lcd_image_pub_;
  ros::Subscriber cam_info_left_sub_;
  ros::Subscriber cam_info_right_sub_;

  tf2_ros::Buffer tfBuffer_;
  std::unique_ptr<tf2_ros::TransformListener> tfListener_;
  std::string body_frame_;
  std::string gnd_truth_topic_;

  std::unique_ptr<LoopClosureDetector> lcd_detector_;
  LoopClosureDetectorParams lcd_params_;
  bool log_output_;
  int verbosity_;
  int frame_count_;

  Timestamp last_time_stamp_;

  image_transport::ImageTransport it_;
  ImageSubscriber left_img_subscriber_;
  ImageSubscriber right_img_subscriber_;

  message_filters::Synchronizer<sync_pol> sync;

  StereoCalibration stereo_calib_;
  StereoMatchingParams stereo_matching_params_;
  bool cam_info_calibrated_; // TODO: not used

  StereoBuffer stereo_buffer_;
  std::vector<LCDTaggedFrame> db_tagged_frames_;

  visualization_msgs::MarkerArray markers_;
  unsigned int marker_id_;
  geometry_msgs::PoseArray pose_array_;

}; // class LCD_ros

} // namespace VIO
