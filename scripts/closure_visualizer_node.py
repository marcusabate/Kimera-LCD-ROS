#!/usr/bin/env python

import rospy
import numpy as np
import tf
import tf2_ros

from lcd_ros.msg import Closure, PoseError
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import TransformStamped, Transform, Pose, PoseArray, \
        Quaternion, Vector3, Point

class ClosureVisualizerNode:
    def __init__(self):
        # NOTE: cache_time is used to tell the transform buffer to listen
        # to all transforms in past time. This is important for loop closure
        # detection as frames can close with any frame in the past
        # TODO: more elegant solution to this problem
        self.cache_time = rospy.get_param("~cache_time")
        self.error_to_scale = rospy.get_param("~error_to_scale")

        self.closure_result_topic = rospy.get_param("~closure_result_topic")
        self.error_topic = rospy.get_param("~error_topic")
        self.body_frame = rospy.get_param("~body_frame")
        self.markers_topic = rospy.get_param("~markers_topic")
        self.poses_topic = rospy.get_param("~poses_topic")

        self.closure_sub = rospy.Subscriber(self.closure_result_topic, \
            Closure, self.closure_cb, queue_size=10)

        self.marker_pub = rospy.Publisher(self.markers_topic, MarkerArray, \
            queue_size=1)
        self.pose_pub = rospy.Publisher(self.poses_topic, PoseArray, \
            queue_size=1)
        self.error_pub = rospy.Publisher(self.error_topic, PoseError, \
            queue_size=1)

        self.buffer = tf2_ros.Buffer(rospy.Time.from_sec(self.cache_time))
        self.listener = tf2_ros.TransformListener(self.buffer)

        self.markers = MarkerArray()
        self.marker_id = 0
        self.pose_array = PoseArray()
        self.pose_array.header.frame_id = "world"

    def closure_cb(self, msg):
        """ ROS callback for closure results from LCD_ros node

            param msg : lcd_ros.Closure message type
        """
        cur_time = msg.cur_time
        ref_time = msg.ref_time
        id_cur = msg.cur_id
        id_ref = msg.ref_id
        ref_to_cur_estim_tf = msg.transform

        # Get absolute ground truth poses for ref and cur frames
        w_to_b_ref_truth_tf = TransformStamped()
        w_to_b_cur_truth_tf = TransformStamped()
        try:
            w_to_b_ref_truth_tf = self.buffer.lookup_transform("world",
                self.body_frame, ref_time)
            w_to_b_cur_truth_tf = self.buffer.lookup_transform("world",
                self.body_frame, cur_time)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as error:
            rospy.logerr("closure_visualizer_node error: " + str(error))
            return

        # Compute transform of estimated current frame in the world frame
        tf_1 = self.tf_msg2mat(w_to_b_ref_truth_tf.transform)
        tf_2 = self.tf_msg2mat(ref_to_cur_estim_tf)
        tf_1_2 = np.dot(tf_1, tf_2)

        w_to_b_cur_estim_tf = TransformStamped()
        w_to_b_cur_estim_tf.header.frame_id = "world"
        w_to_b_cur_estim_tf.header.stamp = cur_time
        w_to_b_cur_estim_tf.child_frame_id = self.body_frame
        w_to_b_cur_estim_tf.transform = self.tf_mat2msg(tf_1_2)

        transforms = [w_to_b_ref_truth_tf,
                      w_to_b_cur_estim_tf,
                      w_to_b_cur_truth_tf]

        # Publish pose array and markers and error
        self.publish_poses(transforms)
        self.publish_markers(transforms)
        self.publish_error(transforms, ref_to_cur_estim_tf, (id_ref, id_cur))

    def publish_poses(self, transforms):
        """ Publish pose array containing poses for cur frame (both estimated
            and ground truth) and the reference frame, in the world frame.

            param transforms : list of TransformStamped messages
        """
        self.pose_array.poses.append(self.transform2pose(
            transforms[0].transform))
        self.pose_array.poses.append(self.transform2pose(
            transforms[1].transform))
        self.pose_array.poses.append(self.transform2pose(
            transforms[2].transform))

        self.pose_array.header.stamp = rospy.Time.now()
        self.pose_pub.publish(self.pose_array)

    def publish_markers(self, transforms):
        """ Publish marker array containing arrows between current frame
            estimated and current frame ground truth.

            param transforms : list of TransformStamped messages
        """
        # Build arrow pointing from ref frame to estimated cur frame
        estim_arrow = Marker()
        estim_arrow.header.stamp = rospy.Time.now()
        estim_arrow.header.frame_id = "world"
        estim_arrow.ns = "LCD"
        estim_arrow.type = Marker.ARROW
        estim_arrow.action = Marker.ADD
        estim_arrow.scale.x = 0.025
        estim_arrow.scale.y = 0.05
        estim_arrow.scale.z = 0.05
        estim_arrow.color.a = 1.0
        estim_arrow.color.g = 1.0
        estim_arrow.points.append(transforms[0].transform.translation)
        estim_arrow.points.append(transforms[1].transform.translation)
        estim_arrow.id = self.marker_id
        self.marker_id += 1

        # Build arrow point from estimated cur frame to truth cur frame
        error_arrow = Marker()
        error_arrow.header.stamp = rospy.Time.now()
        error_arrow.header.frame_id = "world"
        error_arrow.ns = "LCD"
        error_arrow.type = Marker.ARROW
        error_arrow.action = Marker.ADD
        error_arrow.scale.x = 0.025
        error_arrow.scale.y = 0.05
        error_arrow.scale.z = 0.05
        error_arrow.color.a = 1.0
        error_arrow.color.r = 1.0
        error_arrow.points.append(transforms[1].transform.translation)
        error_arrow.points.append(transforms[2].transform.translation)
        error_arrow.id = self.marker_id
        self.marker_id += 1

        # Publish both markers
        self.markers.markers.append(estim_arrow)
        self.markers.markers.append(error_arrow)

        self.marker_pub.publish(self.markers)

    def publish_error(self, transforms, rel_transform_estim, ids):
        """ Publish error in cur-frame estimation compared to ground truth.

            param transforms : list of TransformStamped messages
            param rel_transform_estim : Transform message type
            param ids : tuple containing (ref_id, cur_id)
        """
        error_msg = PoseError()
        error_msg.to_scale = self.error_to_scale
        error_msg.ref_id = ids[0]
        error_msg.cur_id = ids[1]

        rel_transform_truth = self.between_mat(
            self.tf_msg2mat(transforms[0].transform),
            self.tf_msg2mat(transforms[2].transform))

        errors = self.compute_rot_trans_errors(rel_transform_truth,
            self.tf_msg2mat(rel_transform_estim))

        error_msg.rot_error = errors[0]
        error_msg.trans_error = errors[1]

        self.error_pub.publish(error_msg)

    def tf_msg2mat(self, transform_msg):
        """ Convert transform message to numpy 4x4 matrix.

            param transform_msg : Transform message type
        """
        tran = np.array([transform_msg.translation.x,
                         transform_msg.translation.y,
                         transform_msg.translation.z])
        rot = np.array([transform_msg.rotation.x,
                        transform_msg.rotation.y,
                        transform_msg.rotation.z,
                        transform_msg.rotation.w])

        result = tf.transformations.quaternion_matrix(rot)
        result[:3,3] = tran
        return result

    def tf_mat2msg(self, transform_mat):
        """ Convert 4x4 transform numpy matrix to a Transform message type.

            param transform_mat : 4x4 numpy Matrix transformation
        """
        quat = tf.transformations.quaternion_from_matrix(transform_mat)
        trans = tf.transformations.translation_from_matrix(transform_mat)

        result = Transform()
        result.translation = Vector3(trans[0], trans[1], trans[2])
        result.rotation = Quaternion(quat[0], quat[1], quat[2], quat[3])

        return result

    def transform2pose(self, transform):
        """ Convert Transform message type to Pose message type

            param transform : Transform message type
        """
        pose = Pose()

        pose.position.x = transform.translation.x
        pose.position.y = transform.translation.y
        pose.position.z = transform.translation.z
        pose.orientation = transform.rotation

        return pose

    def compute_rot_trans_errors(self, transform_1, transform_2):
        """ Compute rotation and translation errors between two poses.

            param transform_1 : ground truth 4x4 numpy matrix type transform
            param transform_2 : estimated 4x5 numpy matrix type transform
        """
        quat_1 = tf.transformations.quaternion_from_matrix(transform_1)
        quat_2 = tf.transformations.quaternion_from_matrix(transform_2)
        rot_error = np.arccos(2.0*np.square(np.dot(quat_1, quat_2)) - 1)

        trans_1 = tf.transformations.translation_from_matrix(transform_1)[:3]
        trans_2 = tf.transformations.translation_from_matrix(transform_2)[:3]
        if self.error_to_scale:
            norm_1 = np.linalg.norm(trans_1)
            norm_2 = np.linalg.norm(trans_2)
            if norm_2 > 1e-5:
                trans_2 = norm_1 * trans_2 / norm_2
        trans_error_vec = trans_1 - trans_2
        trans_error = np.linalg.norm(trans_error_vec)

        return (rot_error, trans_error)

    def between_mat(self, transform_1, transform_2):
        """ Compute the relative transformation matrix between two transforms

            param transform_1 : first transform matrix, (4x4 numpy matrix)
            param transform_2 : second transform matrix, (4x4 nummpy matrix)
        """
        return np.dot(np.linalg.inv(transform_1), transform_2)


if __name__ == "__main__":
    rospy.init_node("closure_visulaizer_node")
    cvn = ClosureVisualizerNode()
    rospy.spin()
