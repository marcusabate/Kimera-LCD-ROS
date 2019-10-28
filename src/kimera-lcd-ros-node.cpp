/**
  * @file kimera-lcd-ros-node.cpp
  * @brief ROS node for running VIO Loop Closure Detector.
  * @author Marcus Abate
**/

// TODO: need an offline version

#include <memory>

#include <kimera-vio/loopclosure/LoopClosureDetector.h>
#include <kimera-vio/utils/Timer.h>
#include "kimera-lcd-ros/ros-lcd-data-source.h"

#include <gflags/gflags.h>
#include <glog/logging.h>

DEFINE_bool(online_run, true, "RUN VIO ROS online or offline");
DEFINE_bool(log_output, false, "Log output files to disk");

int main(int argc, char** argv){
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  ros::init(argc, argv, "kimera_lcd_ros");

  // Create dataset parser.
  std::shared_ptr<VIO::RosBaseDataProvider> dataset_parser;
  if (FLAGS_online_run) {
    // Running ros online.
    dataset_parser = std::make_shared<VIO::RosLcdDataProvider>();
  } else {
    // Parse rosbag.
    // TODO(marcus): implement Rosbag version
    // dataset_parser = std::make_shared<VIO::RosbagLcdDataProvider>();
  }

  // std::unique_ptr<VIO::LoopClosureDetector> lcd_pipeline = std::make_unique(
  //     dataset_parser->lcd_params_, FLAGS_log_output);

  std::unique_ptr<VIO::LoopClosureDetector> lcd_pipeline = 
      VIO::make_unique<VIO::LoopClosureDetector>(
          VIO::LoopClosureDetectorParams(), FLAGS_log_output);

  std::unique_ptr<std::thread> lcd_thread = VIO::make_unique<std::thread>(
      &VIO::LoopClosureDetector::spin, CHECK_NOTNULL(lcd_pipeline.get()),
      std::ref(dataset_parser->lcd_input_queue_), true);

  lcd_pipeline->registerLcdPgoOutputCallback(
      std::bind(&VIO::RosBaseDataProvider::callbackLoopClosureOutput,
                dataset_parser, std::placeholders::_1));

  auto tic = VIO::utils::Timer::tic();
  bool is_pipeline_successful = false;
  if (FLAGS_parallel_run) {
    auto handle = std::async(std::launch::async, &VIO::RosBaseDataProvider::spin,
                             dataset_parser);
    ros::start();

    while (ros::ok()) {
      continue;
    }
    ROS_INFO("Shutting down ROS and LCD pipeline.");
    ros::shutdown();
    lcd_pipeline->shutdown();
    is_pipeline_successful = handle.get();

    LOG(INFO) << "Joining loop closure thread...";
    if (lcd_thread && lcd_thread->joinable()) {
      lcd_thread->join();
      LOG(INFO) << "Joined loop closure thread...";
    } else {
      LOG_IF(ERROR, parallel_run_) << "Loop closure thread is not joinable...";
    }
  } else {
    is_pipeline_successful = dataset_parser->spin();
  }
  auto spin_duration = VIO::utils::Timer::toc(tic);
  ROS_WARN_STREAM("Spin took: " << spin_duration.count() << " ms.");
  ROS_INFO_STREAM("Pipeline successful? "
                  << (is_pipeline_successful ? "Yes!" : "No!"));
  return is_pipeline_successful ? EXIT_SUCCESS : EXIT_FAILURE;
}
