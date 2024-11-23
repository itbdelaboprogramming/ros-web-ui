#!/usr/bin/env python

import rospy
import json
from std_msgs.msg import String
from geometry_msgs.msg import Pose


class RobotPoseFromString:
    def __init__(self):
        rospy.init_node('robotpose_from_string')

        # Get topics from parameters
        self.string_topic = rospy.get_param('~string_topic', '/string/robotpose')
        self.pose_topic = rospy.get_param('~pose_topic', '/robotpose')

        # Publisher
        self.pose_pub = rospy.Publisher(self.pose_topic, Pose, queue_size=10)

        # Subscriber
        rospy.Subscriber(self.string_topic, String, self.string_callback)

    def string_callback(self, msg):
        pose_dict = json.loads(msg.data)
        pose_msg = Pose()

        # Fill position
        pose_msg.position.x = pose_dict["position"]["x"]
        pose_msg.position.y = pose_dict["position"]["y"]
        pose_msg.position.z = pose_dict["position"]["z"]

        # Fill orientation
        pose_msg.orientation.x = pose_dict["orientation"]["x"]
        pose_msg.orientation.y = pose_dict["orientation"]["y"]
        pose_msg.orientation.z = pose_dict["orientation"]["z"]
        pose_msg.orientation.w = pose_dict["orientation"]["w"]

        self.pose_pub.publish(pose_msg)

if __name__ == '__main__':
    RobotPoseFromString()
    rospy.spin()
