#!/usr/bin/env python

import rospy
import json
from actionlib_msgs.msg import GoalID, GoalStatusArray
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseActionResult
from std_msgs.msg import String


class ActionServer:
    def __init__(self):
        rospy.init_node('action_server')
        
        # Get parameters for all topic names
        self.goal_string_topic = rospy.get_param('~goal_string_topic', '/string/move_base/goal')
        self.cancel_string_topic = rospy.get_param('~cancel_string_topic', '/string/move_base/cancel')
        self.result_string_topic = rospy.get_param('~result_string_topic', '/string/move_base/result')
        self.result_topic = rospy.get_param('~result_topic', '/move_base_convert/result')
        self.goal_topic = rospy.get_param('~goal_topic', '/move_base_convert/goal')
        self.cancel_topic = rospy.get_param('~cancel_topic', '/move_base_convert/cancel')

        # Publisher for string topics
        self.goal_pub = rospy.Publisher(self.goal_string_topic, String, queue_size=10)
        self.cancel_pub = rospy.Publisher(self.cancel_string_topic, String, queue_size=10)
        self.result_pub = rospy.Publisher(self.result_topic, MoveBaseActionResult, queue_size=10)

        # Subscriber for action topics
        rospy.Subscriber(self.goal_topic, MoveBaseActionGoal, self.goal_callback)
        rospy.Subscriber(self.cancel_topic, GoalID, self.cancel_callback)
        rospy.Subscriber(self.result_string_topic, String, self.result_callback)

    def goal_callback(self, msg):
        goal_dict = {
            "header": {
                "seq": msg.header.seq,
                "stamp": {"secs": msg.header.stamp.secs, "nsecs": msg.header.stamp.nsecs},
                "frame_id": msg.header.frame_id,
            },
            "goal_id": {
                "stamp": {"secs": msg.goal_id.stamp.secs, "nsecs": msg.goal_id.stamp.nsecs},
                "id": msg.goal_id.id,
            },
            "goal": {
                "target_pose": {
                    "header": {
                        "seq": msg.goal.target_pose.header.seq,
                        "stamp": {
                            "secs": msg.goal.target_pose.header.stamp.secs,
                            "nsecs": msg.goal.target_pose.header.stamp.nsecs,
                        },
                        "frame_id": msg.goal.target_pose.header.frame_id,
                    },
                    "pose": {
                        "position": {
                            "x": msg.goal.target_pose.pose.position.x,
                            "y": msg.goal.target_pose.pose.position.y,
                            "z": msg.goal.target_pose.pose.position.z,
                        },
                        "orientation": {
                            "x": msg.goal.target_pose.pose.orientation.x,
                            "y": msg.goal.target_pose.pose.orientation.y,
                            "z": msg.goal.target_pose.pose.orientation.z,
                            "w": msg.goal.target_pose.pose.orientation.w,
                        },
                    },
                }
            },
        }
        self.goal_pub.publish(json.dumps(goal_dict))

    def cancel_callback(self, msg):
        cancel_dict = {
            "goal_id": {
                "stamp": {"secs": msg.stamp.secs, "nsecs": msg.stamp.nsecs},
                "id": msg.id,
            }
        }
        self.cancel_pub.publish(json.dumps(cancel_dict))
    
    def result_callback(self, msg):
        result_dict = json.loads(msg.data)
        result_msg = MoveBaseActionResult()

        # Fill header
        result_msg.header.seq = result_dict["header"]["seq"]
        result_msg.header.stamp.secs = result_dict["header"]["stamp"]["secs"]
        result_msg.header.stamp.nsecs = result_dict["header"]["stamp"]["nsecs"]
        result_msg.header.frame_id = result_dict["header"]["frame_id"]

        # Fill status
        status = result_dict["status"]
        result_msg.status.goal_id.stamp.secs = status["goal_id"]["stamp"]["secs"]
        result_msg.status.goal_id.stamp.nsecs = status["goal_id"]["stamp"]["nsecs"]
        result_msg.status.goal_id.id = status["goal_id"]["id"]
        result_msg.status.status = status["status"]
        result_msg.status.text = status["text"]

        # Fill result (empty here but can be extended as needed)
        result_msg.result = None  # Adjust if specific result fields are needed

        self.result_pub.publish(result_msg)


if __name__ == '__main__':
    ActionServer()
    rospy.spin()
