#!/usr/bin/env python3

import rospy
import json
from geometry_msgs.msg import Twist
from std_msgs.msg import String

def string_callback(msg):
    # Deserialize the JSON string into a Python dictionary
    twist_dict = json.loads(msg.data)

    # Convert the dictionary back to a Twist message
    twist_msg = Twist()
    twist_msg.linear.x = twist_dict['linear']['x']
    twist_msg.linear.y = twist_dict['linear']['y']
    twist_msg.linear.z = twist_dict['linear']['z']
    twist_msg.angular.x = twist_dict['angular']['x']
    twist_msg.angular.y = twist_dict['angular']['y']
    twist_msg.angular.z = twist_dict['angular']['z']

    # Publish the decoded Twist message
    twist_decoded_pub.publish(twist_msg)

def string_to_twist_node():
    rospy.init_node('string_to_twist_node', anonymous=True)

    # Subscriber to /twist_string
    rospy.Subscriber('/twist_string', String, string_callback)

    # Publisher to /twist_decoded
    global twist_decoded_pub
    twist_decoded_pub = rospy.Publisher('/twist_decoded', Twist, queue_size=10)

    rospy.spin()

if __name__ == '__main__':
    try:
        string_to_twist_node()
    except rospy.ROSInterruptException:
        pass
