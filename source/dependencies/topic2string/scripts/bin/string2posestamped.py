#!/usr/bin/env python3

import rospy
import json
from topic2string.msg import PoseStampedWithInfo 
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header

def string_callback(msg):
    # Deserialize the JSON string into PoseStampedWithInfo
    pose_data = json.loads(msg.data)

    # Create PoseStampedWithInfo message
    pose_stamped_msg = PoseStampedWithInfo()
    pose_stamped_msg.info.data = pose_data['info']
    pose_stamped_msg.pose.header.stamp.secs = pose_data['pose']['header']['stamp']['secs']
    pose_stamped_msg.pose.header.stamp.nsecs = pose_data['pose']['header']['stamp']['nsecs']
    pose_stamped_msg.pose.header.frame_id = pose_data['pose']['header']['frame_id']
    pose_stamped_msg.pose.pose.position = Point(
        pose_data['pose']['pose']['position']['x'],
        pose_data['pose']['pose']['position']['y'],
        pose_data['pose']['pose']['position']['z']
    )
    pose_stamped_msg.pose.pose.orientation = Quaternion(
        pose_data['pose']['pose']['orientation']['x'],
        pose_data['pose']['pose']['orientation']['y'],
        pose_data['pose']['pose']['orientation']['z'],
        pose_data['pose']['pose']['orientation']['w']
    )

    # Publish the PoseStampedWithInfo message
    pose_stamped_pub.publish(pose_stamped_msg)

def string_to_posestamped_node():
    rospy.init_node('string_to_posestamped_node', anonymous=True)

    # Subscriber to pose_string
    rospy.Subscriber('/pose_string', String, string_callback)

    # Publisher to PoseStampedWithInfo
    global pose_stamped_pub
    pose_stamped_pub = rospy.Publisher('/pose_stamped_with_info', PoseStampedWithInfo, queue_size=10)

    rospy.spin()

if __name__ == '__main__':
    try:
        string_to_posestamped_node()
    except rospy.ROSInterruptException:
        pass
