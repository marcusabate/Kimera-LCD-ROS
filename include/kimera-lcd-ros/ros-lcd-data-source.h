/**
  * @file ros-lcd-data-source.h
  * @brief ROS wrapper for online processing.
  * @author Marcus Abate
**/

#pragma once

#include <string>

#include <ros/ros.h>

#include <kimera-ros/ros-data-source.h>
#include <kimera-ros/rosbag-data-source.h>

#include <kimera-vio/loopclosure/LoopClosureDetector-definitions.h>
#include <kimera-vio/utils/ThreadsafeQueue.h>

namespace VIO {

class RosLcdDataProvider : virtual public VIO::RosDataProvider {
public:
  RosLcdDataProvider();
  virtual ~RosLcdDataProvider() {}
  virtual bool spin() override;

private:
  void publishOutput(
      const VIO::LoopClosureDetectorOutputPayload& payload);
  
  void poseToTransformMsg(const gtsam::Pose3& pose,
      geometry_msgs::Transform* tf);

private:
  ros::Publisher lcd_image_pub_;
  ros::Publisher lcd_closure_pub_;

  std::string closure_image_topic_, closure_result_topic_;
  std::string body_frame_;

  // Queues to store input and output payloads in a thread-safe way.
  ThreadsafeQueue<LoopClosureDetectorInputPayload> lcd_input_queue_;
  ThreadsafeQueue<LoopClosureDetectorOutputPayload> lcd_output_queue_;

}; // class LCD_ros

} // namespace VIO
