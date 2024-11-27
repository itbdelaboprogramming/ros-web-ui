#!/usr/bin/env python

import rospy
import json
from std_msgs.msg import String
from geometry_msgs.msg import Pose


class RobotPoseToString:
    def __init__(self):
        rospy.init_node('robotpose_to_string')

        # Get parameters
        self.pose_topic = rospy.get_param('~pose_topic', '/msd/robotpose')
        self.string_topic = rospy.get_param('~string_topic', '/string/robotpose')
        self.publish_frequency = rospy.get_param('~publish_frequency', 2.0)  # Default 2 Hz

        # Publisher
        self.string_pub = rospy.Publisher(self.string_topic, String, queue_size=10)

        # Last received pose
        self.latest_pose = None

        # Subscriber
        rospy.Subscriber(self.pose_topic, Pose, self.pose_callback)

        # Timer for periodic publishing
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.publish_frequency), self.publish_timer_callback)

    def pose_callback(self, msg):
        # Store the latest pose
        self.latest_pose = msg

    def publish_timer_callback(self, event):
        if self.latest_pose:
            # Convert pose to dictionary
            pose_dict = {
                "position": {
                    "x": self.latest_pose.position.x,
                    "y": self.latest_pose.position.y,
                    "z": self.latest_pose.position.z,
                },
                "orientation": {
                    "x": self.latest_pose.orientation.x,
                    "y": self.latest_pose.orientation.y,
                    "z": self.latest_pose.orientation.z,
                    "w": self.latest_pose.orientation.w,
                },
            }
            # Convert dictionary to JSON string
            pose_string = json.dumps(pose_dict)

            # Publish the string
            # rospy.loginfo(f"Publishing pose to {self.string_topic}")
            self.string_pub.publish(String(data=pose_string))


if __name__ == '__main__':
    try:
        RobotPoseToString()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
