#!/usr/bin/env python3

import rospy
import json
from topic2string.msg import PoseStampedWithInfo
from std_msgs.msg import String

def posestamped_callback(msg):
    # Serialize the PoseStampedWithInfo message into JSON format
    pose_dict = {
        'info': msg.info.data,
        'pose': {
            'header': {
                'stamp': {
                    'secs': msg.pose.header.stamp.secs,
                    'nsecs': msg.pose.header.stamp.nsecs
                },
                'frame_id': msg.pose.header.frame_id
            },
            'pose': {
                'position': {
                    'x': msg.pose.pose.position.x,
                    'y': msg.pose.pose.position.y,
                    'z': msg.pose.pose.position.z
                },
                'orientation': {
                    'x': msg.pose.pose.orientation.x,
                    'y': msg.pose.pose.orientation.y,
                    'z': msg.pose.pose.orientation.z,
                    'w': msg.pose.pose.orientation.w
                }
            }
        }
    }

    pose_json = json.dumps(pose_dict)

    # Publish the JSON string to pose_string
    pose_string_pub.publish(pose_json)

def posestamped_to_string_node():
    rospy.init_node('posestamped_to_string_node', anonymous=True)

    # Subscriber to PoseStampedWithInfo
    rospy.Subscriber('/pose_stamped_with_info', PoseStampedWithInfo, posestamped_callback)

    # Publisher to pose_string
    global pose_string_pub
    pose_string_pub = rospy.Publisher('/pose_string', String, queue_size=10)

    rospy.spin()

if __name__ == '__main__':
    try:
        posestamped_to_string_node()
    except rospy.ROSInterruptException:
        pass
