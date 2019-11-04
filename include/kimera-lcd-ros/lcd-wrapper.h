/**
  * @file ros-lcd-data-source.h
  * @brief ROS wrapper for online processing.
  * @author Marcus Abate
**/

#pragma once

#include <string>

#include <ros/ros.h>

#include <kimera-vio/loopclosure/LoopClosureDetector-definitions.h>
#include <kimera-vio/loopclosure/LoopClosureDetector.h>
#include <kimera-vio/utils/ThreadsafeQueue.h>

#include "kimera_lcd_ros/LcdInputPayload.h"
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>

#include <opencv2/core/mat.hpp>
#include <gtsam/geometry/Pose3.h>

namespace VIO {

struct LcdTaggedFrame {
  LcdTaggedFrame(const VIO::FrameId& frame_id,
                 const VIO::Timestamp& timestamp,
                 const cv::Mat& left_img,
                 const gtsam::Pose3& W_Pose_B_est,
                 const gtsam::Pose3& W_Pose_B_gt)
  : frame_id_(frame_id),
    timestamp_(timestamp),
    left_img_(left_img),
    W_Pose_B_est_(W_Pose_B_est),
    W_Pose_B_gt_(W_Pose_B_gt) {}
  
  VIO::FrameId frame_id_;
  VIO::Timestamp timestamp_;
  cv::Mat left_img_;
  gtsam::Pose3 W_Pose_B_est_;
  gtsam::Pose3 W_Pose_B_gt_;
};

class LcdWrapper {
public:
  LcdWrapper();
  virtual ~LcdWrapper();
  virtual bool spin();

private:
  void start_lcd_thread();
  void callbackLcdInput(const kimera_lcd_ros::LcdInputPayload &msg);
  void callbackLoopClosureOutput(const LoopClosureDetectorOutputPayload &lcd_output);

private:
  void cameraParamsMsg2Struct(const kimera_lcd_ros::CameraParams& msg,
                              VIO::CameraParams* cam_params);
  void stereoMatchingParamsMsg2Struct(const kimera_lcd_ros::StereoMatchingParams& msg,
                                      VIO::StereoMatchingParams* stereo_matching_params);
  cv::Mat readRosImage(const sensor_msgs::Image& img_msg) const;
  void publishLcdOutput(
      const VIO::LoopClosureDetectorOutputPayload &payload);
  void poseToTransformMsg(const gtsam::Pose3& pose,
      geometry_msgs::Transform* tf);
  void shutdown();
  void stopThreads();
  void joinThreads();

private:
  // ROS networking members.
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;  
  ros::Subscriber lcd_input_sub_;
  ros::Publisher lcd_image_pub_;
  ros::Publisher lcd_closure_pub_;

  // ROS frame IDs.
  std::string base_link_frame_id_, map_frame_id_, world_frame_id_;

  // LoopClosureDetector module and thread.
  std::unique_ptr<VIO::LoopClosureDetector> loop_closure_detector_ = {nullptr};
  std::unique_ptr<std::thread> lcd_thread_ = {nullptr};
  std::atomic_bool shutdown_ = {false};

  // Queues to store input and output payloads in a thread-safe way.
  ThreadsafeQueue<LoopClosureDetectorInputPayload> lcd_input_queue_;
  ThreadsafeQueue<LoopClosureDetectorOutputPayload> lcd_output_queue_;

  // Database of input payload frames.
  // TODO(marcus): use a map keyed on farme id instead of a vector
  std::unique_ptr<std::vector<LcdTaggedFrame>> db_frames_;

}; // class LCD_ros

} // namespace VIO
