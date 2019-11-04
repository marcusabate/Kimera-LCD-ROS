#!/usr/bin/env python

import rospy
import tf2_ros

from geometry_msgs.msg import TransformStamped

class TransformConverterNode:
    def __init__(self):
        self.gnd_truth_topic = rospy.get_param("~gnd_truth_topic")
        self.body_frame_id = rospy.get_param("~body_frame_id")
        self.world_frame_id = rospy.get_param("~world_frame_id")

        self.tf_sub = rospy.Subscriber(self.gnd_truth_topic, TransformStamped,\
            self.tf_cb, queue_size=1)

        self.br = tf2_ros.TransformBroadcaster()

    def tf_cb(self, msg):
        msg.header.frame_id = self.world_frame_id
        msg.child_frame_id = self.body_frame_id
        self.br.sendTransform(msg)


if __name__ == "__main__":
    rospy.init_node("transformer_converter_node")
    tcn = TransformConverterNode()
    rospy.spin()
