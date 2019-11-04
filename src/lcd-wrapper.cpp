/**
  * @file ros-lcd-data-source.cpp
  * @brief ROS wrapper for online processing.
  * @author Marcus Abate
**/

#include "kimera-lcd-ros/lcd-wrapper.h"
#include "kimera_lcd_ros/Closure.h"
#include "kimera_lcd_ros/PoseError.h"

#include <kimera-vio/UtilsOpenCV.h>

#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include <gflags/gflags.h>
#include <glog/logging.h>

DEFINE_bool(log_output_lcd, false, "Log output files to disk");
DEFINE_string(lcd_params_path_2, "", "Path to LoopClosureDetector parameters yaml file.");

namespace VIO {

LcdWrapper::LcdWrapper()
    : nh_(),
      nh_private_("~"),
      base_link_frame_id_(""),
      map_frame_id_(""),
      world_frame_id_(""),
      loop_closure_detector_(nullptr),
      lcd_thread_(nullptr),
      shutdown_(false),
      lcd_input_queue_("lcd_input_queue"),
      lcd_output_queue_("lcd_output_queue"),
      db_frames_(nullptr) {
  ROS_INFO(">>>>>>>>>>>>> INITIALIZING LCD ROS WRAPPER >>>>>>>>>>>>>");

  // Get ROS params
  CHECK(nh_private_.getParam("base_link_frame_id", base_link_frame_id_));
  CHECK(nh_private_.getParam("world_frame_id", world_frame_id_));
  CHECK(nh_private_.getParam("map_frame_id", map_frame_id_));
  CHECK(!base_link_frame_id_.empty());
  CHECK(!world_frame_id_.empty());
  CHECK(!map_frame_id_.empty());

  VIO::LoopClosureDetectorParams lcd_params;
  if (FLAGS_lcd_params_path_2.empty()) {
    VLOG(100) << "No LoopClosureDetector parameters specified, using default";
    lcd_params = LoopClosureDetectorParams();
  } else {
    VLOG(100) << "Using user-specified LoopClosureDetector parameters: "
              << FLAGS_lcd_params_path_2;
    lcd_params.parseYAML(FLAGS_lcd_params_path_2);
  }

  loop_closure_detector_ = VIO::make_unique<VIO::LoopClosureDetector>(
      lcd_params, FLAGS_log_output_lcd);

  CHECK(loop_closure_detector_);
  loop_closure_detector_->registerLcdPgoOutputCallback(
      std::bind(&VIO::LcdWrapper::callbackLoopClosureOutput, this,
                std::placeholders::_1));

  start_lcd_thread();

  // Subscribe to input payload topic.
  lcd_input_sub_ = nh_.subscribe("lcd_input", 10, &LcdWrapper::callbackLcdInput, this);

  // Advertise for publishing loop closure events
  lcd_image_pub_ = nh_.advertise<sensor_msgs::Image>("image", 1);
  lcd_closure_pub_ = nh_.advertise<kimera_lcd_ros::Closure>("result", 1);

  db_frames_ = VIO::make_unique<std::vector<LcdTaggedFrame>>();

  ROS_INFO(">>>>>>>>>>>>> LCD ROS WRAPPER INITIALIZED >>>>>>>>>>>>>");
}

LcdWrapper::~LcdWrapper() {
  LOG(INFO) << "LcdWrapper destructor called.";
  if (!shutdown_) {
    shutdown();
  } else {
    LOG(INFO) << "Manual shutdown was requested.";
  }
}

void LcdWrapper::start_lcd_thread() {
  lcd_thread_ = VIO::make_unique<std::thread>(
      &VIO::LoopClosureDetector::spin, CHECK_NOTNULL(loop_closure_detector_.get()),
      std::ref(lcd_input_queue_), true);
}

cv::Mat LcdWrapper::readRosImage(const sensor_msgs::Image& img_msg) const {
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(img_msg);
  } catch (cv_bridge::Exception& exception) {
    ROS_FATAL("cv_bridge exception: %s", exception.what());
    ros::shutdown();
  }

  if (img_msg.encoding == sensor_msgs::image_encodings::BGR8) {
    LOG(WARNING) << "Converting image...";
    cv::cvtColor(cv_ptr->image, cv_ptr->image, cv::COLOR_BGR2GRAY);
  } else {
    CHECK_EQ(cv_ptr->encoding, sensor_msgs::image_encodings::MONO8)
        << "Expected image with MONO8 or BGR8 encoding.";
  }

  return cv_ptr->image;
}

void LcdWrapper::callbackLcdInput(const kimera_lcd_ros::LcdInputPayload& msg) {  
  cv::Mat img_left = readRosImage(msg.stereo_frame.left_image);
  cv::Mat img_right = readRosImage(msg.stereo_frame.right_image);
  
  VIO::FrameId frame_id = msg.stereo_frame.id;
  VIO::Timestamp timestamp = msg.stereo_frame.timestamp;

  VIO::CameraParams left_camera_info, right_camera_info;

  cameraParamsMsg2Struct(msg.stereo_frame.left_camera_params, &left_camera_info);
  cameraParamsMsg2Struct(msg.stereo_frame.right_camera_params, &right_camera_info);

  VIO::StereoMatchingParams stereo_matching_params;
  stereoMatchingParamsMsg2Struct(msg.stereo_frame.stereo_matching_params,
                                 &stereo_matching_params);

  gtsam::Point3 camL_t_camR(msg.stereo_frame.L_Pose_R.position.x,
                            msg.stereo_frame.L_Pose_R.position.y,
                            msg.stereo_frame.L_Pose_R.position.z);
  gtsam::Rot3 camL_R_camR(gtsam::Quaternion(msg.stereo_frame.L_Pose_R.orientation.w,
                                            msg.stereo_frame.L_Pose_R.orientation.x,
                                            msg.stereo_frame.L_Pose_R.orientation.y,
                                            msg.stereo_frame.L_Pose_R.orientation.z));
  gtsam::Pose3 camL_Pose_camR(camL_R_camR, camL_t_camR);

  VIO::StereoFrame stereo_frame(frame_id,
                                timestamp,
                                img_left,
                                left_camera_info,
                                img_right,
                                right_camera_info,
                                camL_Pose_camR,
                                stereo_matching_params);

  stereo_frame.setIsRectified(false);
  stereo_frame.sparseStereoMatching();
  stereo_frame.setIsKeyframe(true);

  gtsam::Point3 trans(msg.w_Pose_b_lkf.position.x,
                      msg.w_Pose_b_lkf.position.y,
                      msg.w_Pose_b_lkf.position.z);
  gtsam::Rot3 rot(gtsam::Quaternion(msg.w_Pose_b_lkf.orientation.w,
                                    msg.w_Pose_b_lkf.orientation.x,
                                    msg.w_Pose_b_lkf.orientation.y,
                                    msg.w_Pose_b_lkf.orientation.z));
  gtsam::Pose3 w_pose_blkf(rot, trans);

  VIO::Timestamp timestamp_kf = msg.timestamp_kf;
  VIO::FrameId cur_kf_id = msg.cur_kf_id;

  lcd_input_queue_.push(
      VIO::LoopClosureDetectorInputPayload(timestamp_kf,
                                           cur_kf_id,
                                           stereo_frame,
                                           w_pose_blkf));

  // TODO(marcus): add tagged frame to db
  db_frames_->push_back(
      LcdTaggedFrame(frame_id,
                     timestamp,
                     img_left,
                     w_pose_blkf,
                     gtsam::Pose3()));
}

void LcdWrapper::callbackLoopClosureOutput(const LoopClosureDetectorOutputPayload &lcd_output) {
  lcd_output_queue_.push(lcd_output);
}

bool LcdWrapper::spin() {
  LOG(INFO) << "LcdWrapper: Starting LcdWrapper Spin...";
  while (ros::ok) {
    // Publish LCD output if any.
    LoopClosureDetectorOutputPayload lcd_output;
    if (lcd_output_queue_.pop(lcd_output)) {
      publishLcdOutput(lcd_output);
    }

    ros::spinOnce();
  }

  ROS_INFO("Spin terminated. Shutting down.");
  // TODO(marcus): shutdown procedure?
  return true;
}

void LcdWrapper::poseToTransformMsg(const gtsam::Pose3& pose,
    geometry_msgs::Transform* tf) {
  gtsam::Quaternion quat = pose.rotation().toQuaternion();

  tf->translation.x = pose.translation()[0];
  tf->translation.y = pose.translation()[1];
  tf->translation.z = pose.translation()[2];
  tf->rotation.x = quat.x();
  tf->rotation.y = quat.y();
  tf->rotation.z = quat.z();
  tf->rotation.w = quat.w();
}

void LcdWrapper::cameraParamsMsg2Struct(const kimera_lcd_ros::CameraParams& msg,
                                        VIO::CameraParams* cam_params) {
  double rate = msg.frame_rate;
  std::vector<uint64_t> resolution = msg.image_size;
  CHECK_EQ(resolution.size(), 2);
 
  cam_params->image_size_ = cv::Size(resolution[0], resolution[1]);
  cam_params->frame_rate_ = 1.0 / rate;

  std::vector<double> intrinsics = msg.intrinsics;
  CHECK_EQ(intrinsics.size(), 4u);
  cam_params->intrinsics_ = intrinsics;
  cam_params->camera_matrix_ = cv::Mat::eye(3, 3, CV_64F);
  cam_params->camera_matrix_.at<double>(0, 0) = intrinsics[0];
  cam_params->camera_matrix_.at<double>(1, 1) = intrinsics[1];
  cam_params->camera_matrix_.at<double>(0, 2) = intrinsics[2];
  cam_params->camera_matrix_.at<double>(1, 2) = intrinsics[3];

  std::vector<double> extrinsics = msg.extrinsics;
  std::vector<double> frame_change = msg.body_Pose_cam;
  CHECK_EQ(extrinsics.size(), 16u);
  CHECK_EQ(frame_change.size(), 16u);

  cv::Mat E_calib = cv::Mat::zeros(4, 4, CV_64F);
  cv::Mat calib2body = cv::Mat::zeros(4, 4, CV_64F);
  for (int k = 0; k < 16; k++) {
    int row = k / 4; // Integer division, truncation of fractional part.
    int col = k % 4;
    E_calib.at<double>(row, col) = extrinsics[k];
    calib2body.at<double>(row, col) = frame_change[k];
  }

  cv::Mat E_body = calib2body * E_calib;

  std::vector<double> extrinsics_body;
  for (int k = 0; k < 16; k++) {
    int row = k / 4; // Integer division, truncation of fractional part.
    int col = k % 4;
    extrinsics_body.push_back(E_body.at<double>(row, col));
  }

  cam_params->body_Pose_cam_ =
      VIO::UtilsOpenCV::poseVectorToGtsamPose3(extrinsics_body);

  cam_params->distortion_model_ = msg.distortion_model;

  std::vector<double> d_coeff = msg.distortion_coefficients;
  cv::Mat distortion_coeff;

  switch (d_coeff.size()) {
    case (4): {
      CHECK_EQ(cam_params->distortion_model_, "radial-tangential");
      distortion_coeff = cv::Mat::zeros(1, 4, CV_64F);
      distortion_coeff.at<double>(0, 0) = d_coeff[0]; // k1
      distortion_coeff.at<double>(0, 1) = d_coeff[1]; // k2
      distortion_coeff.at<double>(0, 3) = d_coeff[2]; // p1 or k3
      distortion_coeff.at<double>(0, 4) = d_coeff[3]; // p2 or k4
      break;
    }
    case (5): {
      CHECK_EQ(cam_params->distortion_model_, "radial-tangential");
      // If given 5 coefficients
      distortion_coeff = cv::Mat::zeros(1, 5, CV_64F);
      for (int k = 0; k < 5; k++) {
        distortion_coeff.at<double>(0, k) = d_coeff[k]; // k1, k2, k3, p1, p2
      }
      break;
    }
    default: {
      ROS_FATAL("Unsupported distortion format.");
    }
  }

  cam_params->distortion_coeff_ = distortion_coeff;
  cam_params->calibration_ =
      gtsam::Cal3DS2(intrinsics[0],                      // fx
                      intrinsics[1],                      // fy
                      0.0,                                // skew
                      intrinsics[2],                      // u0
                      intrinsics[3],                      // v0
                      distortion_coeff.at<double>(0, 0),  //  k1
                      distortion_coeff.at<double>(0, 1),  //  k2
                      distortion_coeff.at<double>(0, 3),  //  p1
                      distortion_coeff.at<double>(0, 4)); //  p2
}

void LcdWrapper::stereoMatchingParamsMsg2Struct(const kimera_lcd_ros::StereoMatchingParams &msg,
                                                VIO::StereoMatchingParams *stereo_matching_params) {
  stereo_matching_params->tolerance_template_matching_ = msg.tol_template_matching;
  stereo_matching_params->nominal_baseline_ = msg.nominal_baseline;
  stereo_matching_params->templ_cols_ = msg.templ_cols;
  stereo_matching_params->templ_rows_ = msg.templ_rows;
  stereo_matching_params->stripe_extra_rows_ = msg.stripe_extra_rows;
  stereo_matching_params->min_point_dist_ = msg.min_point_dist;
  stereo_matching_params->max_point_dist_ = msg.max_point_dist;
  stereo_matching_params->bidirectional_matching_ = msg.bidirectional_matching;
  stereo_matching_params->subpixel_refinement_ = msg.subpixel_refinement;
  stereo_matching_params->equalize_image_ = msg.equalize_image;
  stereo_matching_params->vision_sensor_type_ = msg.vision_sensor_type;
  stereo_matching_params->min_depth_factor_ = msg.min_depth_factor;
  stereo_matching_params->map_depth_factor_ = msg.map_depth_factor;
}

void LcdWrapper::shutdown() {
  LOG_IF(ERROR, shutdown_) << "Shutdown requested, but Pipeline was already "
                              "shutdown.";
  LOG(INFO) << "Shutting down kimera_lcd_ros pipeline.";
  shutdown_ = true;
  stopThreads();
  joinThreads();
  LOG(INFO) << "kimera_lcd_ros destructor finished.";
}

void LcdWrapper::stopThreads() {
  LOG(INFO) << "Stopping loop closure workers and queues...";
  lcd_input_queue_.shutdown();
  if (loop_closure_detector_)
    loop_closure_detector_->shutdown();
}

void LcdWrapper::joinThreads() {
  LOG(INFO) << "Joining loop closure thread...";
  if (lcd_thread_ && lcd_thread_->joinable()) {
    lcd_thread_->join();
    LOG(INFO) << "Joined loop closure thread...";
  } else {
    LOG(ERROR) << "Loop closure thread is not joinable...";
  }
}

void LcdWrapper::publishLcdOutput(const LoopClosureDetectorOutputPayload& lcd_output) {
  // Only publish anything if there is a loop closure detected.
  if (lcd_output.is_loop_closure_) {
    ROS_WARN("Detected loop closure between frames %i and %i.",
        int(lcd_output.id_recent_), int(lcd_output.id_match_));

    CHECK(db_frames_);
    LcdTaggedFrame query_frame = db_frames_->at(lcd_output.id_recent_);
    LcdTaggedFrame match_frame = db_frames_->at(lcd_output.id_match_);

    // Publish an image showing ALL matches between the two images
    // This includes matches that are later filtered out by Lowe + RANSAC
    CHECK(loop_closure_detector_);
    cv::Mat matched_img = loop_closure_detector_->computeAndDrawMatchesBetweenFrames(
        query_frame.left_img_,
        match_frame.left_img_,
        lcd_output.id_recent_,
        lcd_output.id_match_,
        false);

    cv_bridge::CvImage cv_image;
    cv_image.header.stamp = ros::Time::now();
    cv_image.header.frame_id = base_link_frame_id_;
    cv_image.encoding = "bgr8";
    cv_image.image = matched_img;

    sensor_msgs::Image ros_image;
    cv_image.toImageMsg(ros_image);
    lcd_image_pub_.publish(ros_image);

    ros::Time query_time = ros::Time(query_frame.timestamp_ / 1e9);
    ros::Time match_time = ros::Time(match_frame.timestamp_ / 1e9);

    // Publish the loop closure result information
    kimera_lcd_ros::Closure closure_msg;
    closure_msg.cur_id = lcd_output.id_recent_;
    closure_msg.ref_id = lcd_output.id_match_;
    closure_msg.cur_time = query_time;
    closure_msg.ref_time = match_time;
    poseToTransformMsg(lcd_output.relative_pose_, &closure_msg.transform);

    lcd_closure_pub_.publish(closure_msg);
  }
}

} // namespace VIO
