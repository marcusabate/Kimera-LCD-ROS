/**
  * @file ros-lcd-data-source.cpp
  * @brief ROS wrapper for online processing.
  * @author Marcus Abate
**/

#include "kimera-lcd-ros/ros-lcd-data-source.h"

#include <gflags/gflags.h>
#include <glog/logging.h>

namespace VIO {

RosLcdDataProvider::RosLcdDataProvider()
    : DataProvider(),
      it_(nullptr),
      stereo_calib_(),
      nh_(),
      nh_private_("~"),
      lcd_input_queue_("LCD input"),
      lcd_output_queue_("LCD output"),
      left_img_subscriber_(),
      right_img_subscriber_(),
      sync_(nullptr),
      last_timestamp_(0),
      frame_count_(0) {
  ROS_INFO(">>>>>>>>>>>>> INITIALIZING LCD ROS WRAPPER >>>>>>>>>>>>>");

  // Get stereo calibration parmeters
  parseCameraData(&stereo_calib_);

  // Synchronize stereo image callback
  it_ = VIO::make_unique<image_transport::ImageTransport>(nh_);
  DCHECK(it_);

  // Get ROS params
  CHECK(nh_private_.getParam("base_link_frame_id", base_link_frame_id_));
  CHECK(!base_link_frame_id_.empty());
  CHECK(nh_private_.getParam("world_frame_id", world_frame_id_));
  CHECK(!world_frame_id_.empty());
  CHECK(nh_private_.getParam("map_frame_id", map_frame_id_));
  CHECK(!map_frame_id_.empty());
  CHECK(nh_private_.getParam("left_cam_frame_id", left_cam_frame_id_));
  CHECK(!left_cam_frame_id_.empty());
  CHECK(nh_private_.getParam("right_cam_frame_id", right_cam_frame_id_));
  CHECK(!right_cam_frame_id_.empty());

  left_img_subscriber_.subscribe(*it_, "left_cam", 1);
  right_img_subscriber_.subscribe(*it_, "right_cam", 1);

  sync_ = VIO::make_unique<message_filters::Synchronizer<sync_pol>>(
      sync_pol(10), left_img_subscriber_, right_img_subscriber_);
  DCHECK(sync_);
  sync_->registerCallback(
      boost::bind(&RosLcdDataProvider::callbackCamAndProcessStereo, this, _1, _2));

  // Define Callback Queue for Cam Data
  ros::CallbackQueue cam_queue;
  nh_cam_.setCallbackQueue(&cam_queue);

  // Spawn Async Spinner (Running on Custom Queue) for Cam
  ros::AsyncSpinner async_spinner_cam(0, &cam_queue);
  async_spinner_cam.start();

  // Advertise for publishing loop closure events
  lcd_image_pub_ = nh_.advertise<sensor_msgs::Image>(closure_image_topic_, 1);
  lcd_closure_pub_ = nh_.advertise<RosLcdDataProvider::Closure>(closure_result_topic_, 1);

  // Static TFs for left/right cameras
  const gtsam::Pose3 &body_Pose_left_cam =
      stereo_calib_.left_camera_info_.body_Pose_cam_;
  const gtsam::Pose3 &body_Pose_right_cam =
      stereo_calib_.right_camera_info_.body_Pose_cam_;
  publishStaticTf(body_Pose_left_cam,
                  base_link_frame_id_,
                  left_cam_frame_id_);
  publishStaticTf(body_Pose_right_cam,
                  base_link_frame_id_,
                  right_cam_frame_id_);

  ROS_INFO(">>>>>>>>>>>>> LCD ROS WRAPPER INITIALIZED >>>>>>>>>>>>>");
}

bool RosLcdDataProvider::spin() {
  while (ros::ok()) {
    Timestamp timestamp = stereo_buffer_.getEarliestTimestamp();
    if (timestamp <= last_time_stamp_) {
      if (stereo_buffer_.size() != 0) {
        ROS_WARN(
            "Next frame in image buffer is from the same or "
            "earlier time than the last processed frame. Skip "
            "frame.");
        stereo_buffer_.removeNext();
      }
    } else {
      sensor_msgs::ImageConstPtr left_ros_img, right_ros_img;
      stereo_buffer_.extractLatestImages(left_ros_img, right_ros_img);

      // Read ROS images to cv type
        cv::Mat left_image, right_image;

        switch (stereo_matching_params.vision_sensor_type_) {
          case VisionSensorType::STEREO:
            left_image = readRosImage(left_ros_img);
            right_image = readRosImage(right_ros_img);
            break;
          case VisionSensorType::RGBD:  // just use depth to "fake
                                        // right pixel matches" apply
                                        // conversion
            left_image = readRosImage(left_ros_img);
            right_image = readRosDepthImage(right_ros_img);
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

        StereoFrame stereo_frame(frame_count_, timestamp, left_image,
                                 stereo_calib_.left_camera_info_,
                                 right_image,
                                 stereo_calib_.right_camera_info_,
                                 stereo_calib_.camL_Pose_camR_,
                                 stereo_matching_params_);

        stereo_frame.setIsKeyframe(true);  // TODO(marcus): Is this necessary?
        stereo_frame.sparseStereoMatching();

        // TODO(marcus): currently there is no way to provide VIO estimates of pose.
        // Have to add that for this to be useful.
        lcd_input_queue_.push(
            VIO::LoopClosureDetectorInputPayload(timestamp, frame_count_,
                                                 stereo_frame, gtsam::Pose3()));

        last_time_stamp_ = timestamp;
        frame_count_++;
      }
    }

    // Publish LCD output if any.
    LoopClosureDetectorOutputPayload lcd_output;
    if (lcd_output_queue_.pop(lcd_output)) {
      publishOutput(lcd_output);
    }

    ros::spinOnce();
  }

  ROS_INFO("Spin terminated. Shutting down.");
  lcd_input_queue_.shutdown();
  lcd_output_queue_.shutdown();

  return true;
}

void RosLcdDataProvider::poseToTransformMsg(const gtsam::Pose3& pose,
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

void RosLcdDataProvider::publishOutput(const LoopClosureDetectorOutputPayload& lcd_output) {
  // publishLcdOutput(lcd_output);  // TODO(marcus): Add this in once it makes sense to do so

  // Only publish anything if there is a loop closure detected.
  if (lcd_output.is_loop_closure_) {
    ROS_WARN("Detected loop closure between frames %i and %i.",
        int(lcd_output.id_recent_), int(lcd_output.id_match_));

    LCDTaggedFrame query_frame = db_tagged_frames_[lcd_output.id_recent_];
    LCDTaggedFrame match_frame = db_tagged_frames_[lcd_output.id_match_];

    // Publish an image showing ALL matches between the two images
    // This includes matches that are later filtered out by Lowe + RANSAC
    cv::Mat matched_img = lcd_detector_->computeAndDrawMatchesBetweenFrames(
        query_frame.left_img_,
        match_frame.left_img_,
        lcd_output.id_recent_,
        lcd_output.id_match_,
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
    RosLcdDataProvider::Closure closure_msg;
    closure_msg.cur_id = lcd_output.id_recent_;
    closure_msg.ref_id = lcd_output.id_match_;
    closure_msg.cur_time = query_time;
    closure_msg.ref_time = match_time;
    poseToTransformMsg(lcd_output.relative_pose_, &closure_msg.transform);

    lcd_closure_pub_.publish(closure_msg);
  }
}

} // namespace VIO
