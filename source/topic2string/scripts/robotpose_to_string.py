#!/usr/bin/env python

import rospy
import json
from std_msgs.msg import String
from geometry_msgs.msg import Pose


class RobotPoseToString:
    def __init__(self):
        rospy.init_node('robotpose_to_string')

        # Get topics from parameters
        self.pose_topic = rospy.get_param('~pose_topic', '/new_robot_pose')
        self.string_topic = rospy.get_param('~string_topic', '/robot_pose_string')

        # Publisher
        self.string_pub = rospy.Publisher(self.string_topic, String, queue_size=10)

        # Subscriber
        rospy.Subscriber(self.pose_topic, Pose, self.pose_callback)

    def pose_callback(self, msg):
        pose_dict = {
            "position": {
                "x": msg.position.x,
                "y": msg.position.y,
                "z": msg.position.z,
            },
            "orientation": {
                "x": msg.orientation.x,
                "y": msg.orientation.y,
                "z": msg.orientation.z,
                "w": msg.orientation.w,
            },
        }
        pose_string = json.dumps(pose_dict)
        self.string_pub.publish(String(data=pose_string))


if __name__ == '__main__':
    RobotPoseToString()
    rospy.spin()
