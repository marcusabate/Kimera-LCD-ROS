/** LCD_ros_node
  *
  * @file LCD_ros_node.cpp
  * @brief ROS node for running VIO Loop Closure Detector
  * @author Marcus Abate
  *
**/

// TODO: need an offline version

#include "lcd_ros/LCD_ros.h"

#include <gflags/gflags.h>
#include <glog/logging.h>

// namespace VIO {

int main(int argc, char** argv){
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  ros::init(argc, argv, "LCD_ros_node");
  ros::NodeHandle nh;

  VIO::LCD_ros ros_wrapper;

  bool is_pipeline_successful = false;

  auto tic = VIO::utils::Timer::tic();
  is_pipeline_successful = ros_wrapper.spin();
  auto spin_duration = VIO::utils::Timer::toc(tic);

  LOG(WARNING) << "Spin took: " << spin_duration.count() << " ms.";

  return 0;
}

// } // namespace VIO
