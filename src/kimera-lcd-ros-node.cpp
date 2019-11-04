/**
  * @file kimera-lcd-ros-node.cpp
  * @brief ROS node for running VIO Loop Closure Detector.
  * @author Marcus Abate
**/

// TODO: need an offline version

#include <memory>

#include <kimera-vio/utils/Timer.h>
#include "kimera-lcd-ros/lcd-wrapper.h"

#include <gflags/gflags.h>
#include <glog/logging.h>

int main(int argc, char** argv){
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  ros::init(argc, argv, "kimera_lcd_ros");

  auto tic = VIO::utils::Timer::tic();
  bool is_pipeline_successful = false;

  ros::start();

  VIO::LcdWrapper ros_wrapper;

  while (ros::ok) {
    ros_wrapper.spin();
  }

  ROS_INFO("Shutting down ROS and LCD pipeline.");
  ros::shutdown();
  // TODO(marcus): shutdown ros_wrapper
  is_pipeline_successful = true;

  auto spin_duration = VIO::utils::Timer::toc(tic);
  ROS_WARN_STREAM("Spin took: " << spin_duration.count() << " ms.");
  ROS_INFO_STREAM("Pipeline successful? "
                  << (is_pipeline_successful ? "Yes!" : "No!"));
  return is_pipeline_successful ? EXIT_SUCCESS : EXIT_FAILURE;
}
