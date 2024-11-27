#!/usr/bin/env python

import rospy
import json
from actionlib_msgs.msg import GoalID, GoalStatusArray
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseActionResult
from std_msgs.msg import String


class ActionClient:
    def __init__(self):
        rospy.init_node('action_client')
        
        # Load topic names from ROS parameters
        self.goal_topic = rospy.get_param('~goal_topic', '/move_base/goal')
        self.cancel_topic = rospy.get_param('~cancel_topic', '/move_base/cancel')
        self.goal_string_topic = rospy.get_param('~goal_string_topic', '/string/move_base/goal')
        self.cancel_string_topic = rospy.get_param('~cancel_string_topic', '/string/move_base/cancel')
        self.result_topic = rospy.get_param('~result_topic', '/move_base/result')
        self.result_string_topic = rospy.get_param('~result_string_topic', '/string/move_base/result')

        # Publishers for action server topics
        self.goal_pub = rospy.Publisher(self.goal_topic, MoveBaseActionGoal, queue_size=10)
        self.cancel_pub = rospy.Publisher(self.cancel_topic, GoalID, queue_size=10)
        self.result_string_pub = rospy.Publisher(self.result_string_topic, String, queue_size=10)

        # Subscribers for string topics
        rospy.Subscriber(self.goal_string_topic, String, self.goal_callback)
        rospy.Subscriber(self.cancel_string_topic, String, self.cancel_callback)
        rospy.Subscriber(self.result_topic, MoveBaseActionResult, self.result_callback)

    def goal_callback(self, msg):
        goal_dict = json.loads(msg.data)
        goal_msg = MoveBaseActionGoal()

        # Fill header
        goal_msg.header.seq = goal_dict["header"]["seq"]
        goal_msg.header.stamp.secs = goal_dict["header"]["stamp"]["secs"]
        goal_msg.header.stamp.nsecs = goal_dict["header"]["stamp"]["nsecs"]
        goal_msg.header.frame_id = goal_dict["header"]["frame_id"]

        # Fill goal_id
        goal_msg.goal_id.stamp.secs = goal_dict["goal_id"]["stamp"]["secs"]
        goal_msg.goal_id.stamp.nsecs = goal_dict["goal_id"]["stamp"]["nsecs"]
        goal_msg.goal_id.id = goal_dict["goal_id"]["id"]

        # Fill goal
        pose = goal_dict["goal"]["target_pose"]["pose"]
        goal_msg.goal.target_pose.pose.position.x = pose["position"]["x"]
        goal_msg.goal.target_pose.pose.position.y = pose["position"]["y"]
        goal_msg.goal.target_pose.pose.position.z = pose["position"]["z"]
        goal_msg.goal.target_pose.pose.orientation.x = pose["orientation"]["x"]
        goal_msg.goal.target_pose.pose.orientation.y = pose["orientation"]["y"]
        goal_msg.goal.target_pose.pose.orientation.z = pose["orientation"]["z"]
        goal_msg.goal.target_pose.pose.orientation.w = pose["orientation"]["w"]

        # Fill target_pose.header
        target_header = goal_dict["goal"]["target_pose"]["header"]
        goal_msg.goal.target_pose.header.seq = target_header["seq"]
        goal_msg.goal.target_pose.header.stamp.secs = target_header["stamp"]["secs"]
        goal_msg.goal.target_pose.header.stamp.nsecs = target_header["stamp"]["nsecs"]
        goal_msg.goal.target_pose.header.frame_id = target_header["frame_id"]

        self.goal_pub.publish(goal_msg)

    def cancel_callback(self, msg):
        cancel_dict = json.loads(msg.data)
        cancel_msg = GoalID()
        cancel_msg.stamp.secs = cancel_dict["goal_id"]["stamp"]["secs"]
        cancel_msg.stamp.nsecs = cancel_dict["goal_id"]["stamp"]["nsecs"]
        cancel_msg.id = cancel_dict["goal_id"]["id"]
        self.cancel_pub.publish(cancel_msg)

    def result_callback(self, msg):
        result_dict = {
            "header": {
                "seq": msg.header.seq,
                "stamp": {"secs": msg.header.stamp.secs, "nsecs": msg.header.stamp.nsecs},
                "frame_id": msg.header.frame_id,
            },
            "status": {
                "goal_id": {
                    "stamp": {"secs": msg.status.goal_id.stamp.secs, "nsecs": msg.status.goal_id.stamp.nsecs},
                    "id": msg.status.goal_id.id,
                },
                "status": msg.status.status,
                "text": msg.status.text,
            },
            "result": {},  # Adjust as needed for result fields
        }
        self.result_string_pub.publish(json.dumps(result_dict))

if __name__ == '__main__':
    ActionClient()
    rospy.spin()
