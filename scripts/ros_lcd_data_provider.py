#! /usr/bin/env python

import rospy
import numpy as np

import message_filters
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose
import tf.transformations as transformations
from kimera_lcd_ros.msg import LcdInputPayload, StereoFrame, StereoMatchingParams, CameraParams

class RosLcdDataProviderNode:
    def __init__(self):

        self.image_sub_left = message_filters.Subscriber("left_cam", Image)
        self.image_sub_right = message_filters.Subscriber("right_cam", Image)

        self.ts = message_filters.TimeSynchronizer(
            [self.image_sub_left, self.image_sub_right], 10)
        self.ts.registerCallback(self.camera_cb)

        self.lcd_input_pub = rospy.Publisher("lcd_input", LcdInputPayload, queue_size=1)

        self.L_Pose_R = Pose()
        self.left_camera_params = CameraParams()
        self.right_camera_params = CameraParams()
        self.stereo_matching_params = StereoMatchingParams()
        
        self.frame_id = 1
        self.cur_kf_id = 1

        self.stereoMatchingParamInit()
        self.parse_camera_data()
        
    def camera_cb(self, left_msg, right_msg):
        """ ROS callback for time-synchronized camera images. Left and right camera 
            messages are synchronized and stored for use in LcdIputPayloads.

            param left_msg: A sensor_msgs.Image message representing the left camera.
            param right_msg: A sensor_msgs.Image message representing the right camera.
        """
        if self.frame_id % 5 == 0:
            stereo_frame = StereoFrame()
            stereo_frame.header = left_msg.header
            stereo_frame.timestamp = RosLcdDataProviderNode.rosTimeToUint(left_msg.header.stamp)
            stereo_frame.id = self.cur_kf_id
            stereo_frame.left_image = left_msg
            stereo_frame.right_image = right_msg
            stereo_frame.left_camera_params = self.left_camera_params
            stereo_frame.right_camera_params = self.right_camera_params
            stereo_frame.stereo_matching_params = self.stereo_matching_params
            stereo_frame.L_Pose_R = self.L_Pose_R

            lcd_input_payload = LcdInputPayload()
            lcd_input_payload.header = stereo_frame.header
            lcd_input_payload.timestamp_kf = stereo_frame.timestamp
            lcd_input_payload.cur_kf_id = self.cur_kf_id
            lcd_input_payload.stereo_frame = stereo_frame
            lcd_input_payload.w_Pose_b_lkf = Pose()

            self.lcd_input_pub.publish(lcd_input_payload)

            self.cur_kf_id += 1
        self.frame_id += 1

    def parse_camera_data(self):
        """ Retrieve camera data from calibration files. """
        rate = rospy.get_param("~camera_rate_hz")
        resolution = rospy.get_param("~camera_resolution")
        body_Pose_cam = body_Pose_cam = rospy.get_param("~calibration_to_body_frame")
        distortion_model = rospy.get_param("~distortion_model")

        for i in range(2):
            camera_name = ""
            camera_param_i = CameraParams()
            camera_param_i.frame_rate = rate
            camera_param_i.image_size = resolution
            camera_param_i.body_Pose_cam = body_Pose_cam
            camera_param_i.distortion_model = distortion_model

            if i == 0:
                camera_name = "left_camera_"
            else:
                camera_name = "right_camera_"
            
            camera_param_i.intrinsics = rospy.get_param("~" + camera_name + "intrinsics")
            camera_param_i.extrinsics = rospy.get_param("~" + camera_name + "extrinsics")
            camera_param_i.distortion_coefficients = rospy.get_param("~" + camera_name + "distortion_coefficients")
        
            if (i == 0):
                self.left_camera_params = camera_param_i
            else:
                self.right_camera_params = camera_param_i

        L_Pose_R_mat = np.dot(np.linalg.inv(np.reshape(self.left_camera_params.extrinsics, (4,4))),
                              np.reshape(self.right_camera_params.extrinsics, (4,4)))

        L_Pose_R_trans = L_Pose_R_mat[:3,3]
        self.L_Pose_R.position.x = L_Pose_R_trans[0]
        self.L_Pose_R.position.y = L_Pose_R_trans[1]
        self.L_Pose_R.position.z = L_Pose_R_trans[2]

        L_Pose_R_mat[:3,3] = np.array([0, 0, 0])
        L_Pose_R_quat = transformations.quaternion_from_matrix(L_Pose_R_mat)

        self.L_Pose_R.orientation.x = L_Pose_R_quat[0]
        self.L_Pose_R.orientation.y = L_Pose_R_quat[1]
        self.L_Pose_R.orientation.z = L_Pose_R_quat[2]
        self.L_Pose_R.orientation.w = L_Pose_R_quat[3]
    
    @staticmethod
    def rosTimeToUint(stamp):
        """
        """
        return stamp.to_nsec()

    def stereoMatchingParamInit(self):
        """
        """
        self.stereo_matching_params.tol_template_matching = 0.15
        self.stereo_matching_params.nominal_baseline = 0.11
        self.stereo_matching_params.templ_cols = 101
        self.stereo_matching_params.templ_rows = 11
        self.stereo_matching_params.stripe_extra_rows = 0
        self.stereo_matching_params.min_point_dist = 0.5
        self.stereo_matching_params.max_point_dist = 10
        self.stereo_matching_params.bidirectional_matching = False
        self.stereo_matching_params.subpixel_refinement = False
        self.stereo_matching_params.equalize_image = False
        self.stereo_matching_params.vision_sensor_type = 0
        self.stereo_matching_params.min_depth_factor = 0.3
        self.stereo_matching_params.map_depth_factor = 0.001


if __name__ == "__main__":
    rospy.init_node("ros_lcd_data_provider_node")
    ros_lcd_data_provider_node = RosLcdDataProviderNode()
    rospy.spin()
