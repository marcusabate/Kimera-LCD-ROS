/** LCD_ros
  *
  * @file LCD_ros.cpp
  * @brief ROS wrapper for VIO Loop Closure Detector and rosbags as datasource
  * @author Marcus Abate
  *
**/

#include "lcd_ros/LCD_ros.h"
#include "lcd_ros/Closure.h"
#include <gflags/gflags.h>
#include <glog/logging.h>

DECLARE_string(vocabulary_path);

namespace VIO {

LCD_ros::LCD_ros()
    : nh_(ros::NodeHandle()),
      nh_priv_(ros::NodeHandle("~")),
      frame_count_(0),
      last_time_stamp_(0),
      it_(nh_),
      left_img_subscriber_(it_, "left_cam", 1),
      right_img_subscriber_(it_, "right_cam", 1),
      sync(sync_pol(10), left_img_subscriber_, right_img_subscriber_) {
  ROS_INFO(">>>>>>>>>>>>> INITIALIZING LCD ROS WRAPPER >>>>>>>>>>>>>");

  // Get and declare Loop Closure Detector parameters
  nh_priv_.getParam("log_output", log_output_);
  nh_priv_.getParam("body_frame", body_frame_);

  std::string path_to_vocab;
  nh_priv_.getParam("path_to_vocab", path_to_vocab);
  FLAGS_vocabulary_path = path_to_vocab;

  nh_priv_.getParam("closure_result_topic", closure_result_topic_);
  nh_priv_.getParam("closure_image_topic", closure_image_topic_);

  // Get stereo calibration parmeters
  parseCameraData(&stereo_calib_);

  // Stereo matching parameters
  std::string tracker_params_path;
  nh_.getParam("/tracker_params_path", tracker_params_path);
  VioFrontEndParams frontend_params;
  // TODO: get rid of requirement to have frontend_params
  frontend_params.parseYAML(tracker_params_path);
  stereo_matching_params_ = frontend_params.getStereoMatchingParams();

  // Synchronize stereo image callback
  sync.registerCallback(boost::bind(&LCD_ros::callbackCamAndProcessStereo,
      this, _1, _2) );

  // Define callback queue for camera data
  ros::CallbackQueue cam_queue;
  nh_.setCallbackQueue(&cam_queue);

  // Spawn Asyn spinner for camera data
  ros::AsyncSpinner async_spinner_cam(0, &cam_queue);
  async_spinner_cam.start();

  // Subscribe to info topics to get camera parameters on the fly
  cam_info_left_sub_ = nh_.subscribe("left_cam/camera_info", 1,
      &LCD_ros::leftCameraInfoCallback, this);
  cam_info_right_sub_ = nh_.subscribe("right_cam/camera_info", 1,
      &LCD_ros::rightCameraInfoCallback, this);

  // Advertise for publishing loop closure events
  lcd_image_pub_ = nh_.advertise<sensor_msgs::Image>(closure_image_topic_, 1);
  lcd_closure_pub_ = nh_.advertise<lcd_ros::Closure>(closure_result_topic_, 1);

  db_tagged_frames_.clear();

  // Start the loop detector
  std::string path_to_params;
  nh_.getParam("/lcd_params_path", path_to_params);
  lcd_params_ = LoopClosureDetectorParams();
  lcd_params_.parseYAML(path_to_params);

  lcd_detector_ = VIO::make_unique<LoopClosureDetector>(lcd_params_,
      log_output_);

  ROS_INFO(">>>>>>>>>>>>> LCD ROS WRAPPER INITIALIZED >>>>>>>>>>>>>");
}

bool LCD_ros::spin() {
  while (ros::ok()) {
    Timestamp timestamp = stereo_buffer_.getEarliestTimestamp();

    if (timestamp <= last_time_stamp_) {
      if (stereo_buffer_.size() != 0) {
        ROS_WARN("Skipping frame; same or earlier than last processed.");
        stereo_buffer_.removeNext();
      }
    }
    else {
      sensor_msgs::ImageConstPtr left_ros_img, right_ros_img;
      stereo_buffer_.extractLatestImages(left_ros_img, right_ros_img);

      if (left_ros_img.get() != nullptr && right_ros_img.get() != nullptr) {
        cv::Mat left_image, right_image;
        switch(stereo_matching_params_.vision_sensor_type_){
            case VisionSensorType::STEREO :
                // no conversion
  				      left_image = readRosImage(left_ros_img);
  				      right_image = readRosImage(right_ros_img);
                break;
            case VisionSensorType::RGBD : // just use depth to "fake right pixel matches"
                // apply conversion
  				      left_image = readRosRGBImage(left_ros_img);
  				      right_image = readRosDepthImage(right_ros_img);
                break;
              break;
            default:
              LOG(FATAL) << "vision sensor type not recognised.";
              break;
        }

        // Add frame to our database for later visualization
        // Transform is added later
        LCDTaggedFrame tagged_frame;
        tagged_frame.left_img_ = left_image;
        tagged_frame.timestamp_ = timestamp;
        db_tagged_frames_.push_back(tagged_frame);

        StereoFrame stereo_frame(
            frame_count_,
            timestamp,
            left_image,
            stereo_calib_.left_camera_info_,
            right_image,
            stereo_calib_.right_camera_info_,
            stereo_calib_.camL_Pose_camR_,
            stereo_matching_params_);

        stereo_frame.setIsKeyframe(true);
        stereo_frame.sparseStereoMatching();

        std::shared_ptr<LoopClosureDetectorInputPayload> input_payload =
            std::make_shared<LoopClosureDetectorInputPayload>(
                timestamp, stereo_frame);

        auto tic = VIO::utils::Timer::tic();

        LoopClosureDetectorOutputPayload output_payload =
            lcd_detector_->spinOnce(input_payload);

        auto toc = VIO::utils::Timer::toc(tic);
        ROS_INFO("Processed frame %i in %i ms.", frame_count_, int(toc.count()));

        publishOutput(output_payload);

        last_time_stamp_ = timestamp;
        frame_count_++;
      }
    }
    ros::spinOnce();
  }

  ROS_INFO("Spin terminated. Shutting down.");
  return true;
}

cv::Mat LCD_ros::readRosImage(const sensor_msgs::ImageConstPtr& img_msg) {
  // Use cv_bridge to read ros image to cv::Mat
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(img_msg);
  }
  catch(cv_bridge::Exception& exception) {
    ROS_FATAL("cv_bridge exception: %s", exception.what());
    ros::shutdown();
  }
  return cv_ptr->image; // Return cv::Mat
}

cv::Mat LCD_ros::readRosRGBImage(const sensor_msgs::ImageConstPtr& img_msg) {
  // Use cv_bridge to read ros image to cv::Mat
  cv::Mat img_rgb = LCD_ros::readRosImage(img_msg);
  cv::cvtColor(img_rgb, img_rgb, cv::COLOR_BGR2GRAY); //CV_RGB2GRAY);
  return img_rgb; // Return cv::Mat
}

cv::Mat LCD_ros::readRosDepthImage(const sensor_msgs::ImageConstPtr& img_msg) {
  // Use cv_bridge to read ros image to cv::Mat
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(
        img_msg,sensor_msgs::image_encodings::TYPE_16UC1);
  }
  catch(cv_bridge::Exception& exception) {
    ROS_FATAL("cv_bridge exception: %s", exception.what());
    ros::shutdown();
  }
  cv::Mat img_depth = cv_ptr->image;
  if(img_depth.type()!=CV_16UC1)
        img_depth.convertTo(img_depth,CV_16UC1); // mDepthMapFactor);
  return img_depth; // Return cv::Mat
}

bool LCD_ros::parseCameraData(StereoCalibration* stereo_calib) {
  // Parse camera calibration info (from param server)

  // Rate
  double rate;
  nh_.getParam("/camera_rate_hz", rate);

  // Resoltuion
  std::vector<int> resolution;
  nh_.getParam("/camera_resolution", resolution);

  // Get distortion/intrinsics/extrinsics for each camera
  for (int i = 0; i < 2; i++){
    std::string camera_name;
    CameraParams camera_param_i;
    // Fill in rate and resolution
    camera_param_i.image_size_ = cv::Size(resolution[0], resolution[1]);
    camera_param_i.frame_rate_ = 1.0 / rate; // Terminology wrong but following rest of the repo

    if (i == 0) {
      camera_name = "left_camera_";
    } else {
      camera_name = "right_camera_";
    }
    // Parse intrinsics (camera matrix)
    std::vector<double> intrinsics;
    nh_.getParam("/" + camera_name + "intrinsics", intrinsics);
    camera_param_i.intrinsics_ = intrinsics;
    // Conver intrinsics to camera matrix (OpenCV format)
    camera_param_i.camera_matrix_ = cv::Mat::eye(3, 3, CV_64F);
    camera_param_i.camera_matrix_.at<double>(0, 0) = intrinsics[0];
    camera_param_i.camera_matrix_.at<double>(1, 1) = intrinsics[1];
    camera_param_i.camera_matrix_.at<double>(0, 2) = intrinsics[2];
    camera_param_i.camera_matrix_.at<double>(1, 2) = intrinsics[3];

    // Parse extrinsics (rotation and translation)
    std::vector<double> extrinsics;
    std::vector<double> frame_change; // encode calibration frame to body frame
    CHECK(nh_.getParam("/" + camera_name + "extrinsics", extrinsics));
    CHECK(nh_.getParam("/calibration_to_body_frame", frame_change));
    // Place into matrix
    // 4 4 is hardcoded here because currently only accept extrinsic input
    // in homoegeneous format [R T ; 0 1]
    cv::Mat E_calib = cv::Mat::zeros(4, 4, CV_64F);
    cv::Mat calib2body = cv::Mat::zeros(4, 4, CV_64F);
    for (int k = 0; k < 16; k++) {
      int row = k / 4;
      int col = k % 4;
      E_calib.at<double>(row, col) = extrinsics[k];
      calib2body.at<double>(row, col) = frame_change[k];
    }

    // TODO: Check frames convention!
    cv::Mat E_body = calib2body * E_calib; // Extrinsics in body frame

    // restore back to vector form
    std::vector<double> extrinsics_body;
    for (int k = 0; k < 16; k++) {
      int row = k / 4;
      int col = k % 4;
      extrinsics_body.push_back(E_body.at<double>(row, col));
    }

    camera_param_i.body_Pose_cam_ = UtilsOpenCV::Vec2pose(extrinsics_body, 4, 4);

    // Distortion model
    std::string distortion_model;
    nh_.getParam("/distortion_model", distortion_model);
    camera_param_i.distortion_model_ = distortion_model;

    // Parse distortion
    std::vector<double> d_coeff;
    nh_.getParam("/" + camera_name + "distortion_coefficients", d_coeff);
    cv::Mat distortion_coeff;

    switch (d_coeff.size()) {
      case(4): // if given 4 coefficients
        ROS_INFO("using radtan or equidistant distortion model (4 coefficients) for camera %d", i);
        distortion_coeff = cv::Mat::zeros(1, 4, CV_64F);
        distortion_coeff.at<double>(0,0) = d_coeff[0]; // k1
        distortion_coeff.at<double>(0,1) = d_coeff[1]; // k2
        distortion_coeff.at<double>(0,3) = d_coeff[2]; // p1 or k3
        distortion_coeff.at<double>(0,4) = d_coeff[3]; // p2 or k4
        break;

      case(5): // if given 5 coefficients
        ROS_INFO("using radtan distortion model (5 coefficients) for camera %d", i);
        distortion_coeff = cv::Mat::zeros(1, 5, CV_64F);
        for (int k = 0; k < 5; k++) {
          distortion_coeff.at<double>(0, k) = d_coeff[k]; // k1, k2, k3, p1, p2
        }
        break;

      default: // otherwise
        ROS_FATAL("Unsupported distortion format");
    }

    camera_param_i.distortion_coeff_ = distortion_coeff;

    // TODO add skew (can add switch statement when parsing intrinsics)
    camera_param_i.calibration_ = gtsam::Cal3DS2(intrinsics[0], // fx
        intrinsics[1], // fy
        0.0,           // skew
        intrinsics[2], // u0
        intrinsics[3], // v0
        distortion_coeff.at<double>(0,0),  //  k1
        distortion_coeff.at<double>(0,1),  //  k2
        distortion_coeff.at<double>(0,3),  //  p1
        distortion_coeff.at<double>(0,4)); //  p2

    if (i == 0){
      stereo_calib->left_camera_info_ = camera_param_i;
    } else {
      stereo_calib->right_camera_info_ = camera_param_i;
    }

  }

  // Calculate the pose of right camera relative to the left camera
  stereo_calib->camL_Pose_camR_ = (stereo_calib->left_camera_info_.body_Pose_cam_).between(
                                    stereo_calib->right_camera_info_.body_Pose_cam_);

  ROS_INFO("Parsed stereo camera calibration");
  return true;
}

void LCD_ros::callbackCamAndProcessStereo(
    const sensor_msgs::ImageConstPtr& msgLeft,
    const sensor_msgs::ImageConstPtr& msgRight) {
  stereo_buffer_.addStereoFrame(msgLeft, msgRight);
}

void LCD_ros::leftCameraInfoCallback(
    const sensor_msgs::CameraInfo::ConstPtr& msg) {
  // TODO real time camera parameter calibration
}

void LCD_ros::rightCameraInfoCallback(
    const sensor_msgs::CameraInfo::ConstPtr& msg) {
  // TODO real time camera parameter calibration
}

void LCD_ros::poseToTransformMsg(const gtsam::Pose3& pose,
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

void LCD_ros::publishOutput(const LoopClosureDetectorOutputPayload& payload) {
  // Only publish anything if there is a loop closure detected.
  if (payload.is_loop_) {
    ROS_WARN("Detected loop closure between frames %i and %i.",
        int(payload.id_recent_), int(payload.id_match_));

    LCDTaggedFrame query_frame = db_tagged_frames_[payload.id_recent_];
    LCDTaggedFrame match_frame = db_tagged_frames_[payload.id_match_];

    // Publish an image showing ALL matches between the two images
    // This includes matches that are later filtered out by Lowe + RANSAC
    cv::Mat matched_img = lcd_detector_->computeAndDrawMatchesBetweenFrames(
        query_frame.left_img_,
        match_frame.left_img_,
        payload.id_recent_,
        payload.id_match_,
        false);

    cv_bridge::CvImage cv_image;
    cv_image.header.stamp = ros::Time::now();
    cv_image.header.frame_id = body_frame_;
    cv_image.encoding = "bgr8";
    cv_image.image = matched_img;

    sensor_msgs::Image ros_image;
    cv_image.toImageMsg(ros_image);
    lcd_image_pub_.publish(ros_image);

    ros::Time query_time = ros::Time(query_frame.timestamp_ / 1e9);
    ros::Time match_time = ros::Time(match_frame.timestamp_ / 1e9);

    // Publish the loop closure result information
    lcd_ros::Closure closure_msg;
    closure_msg.cur_id = payload.id_recent_;
    closure_msg.ref_id = payload.id_match_;
    closure_msg.cur_time = query_time;
    closure_msg.ref_time = match_time;
    poseToTransformMsg(payload.relative_pose_, &closure_msg.transform);

    lcd_closure_pub_.publish(closure_msg);
  }
}

} // namespace VIO
