#!/usr/bin/env python3

import rospy
import json
from geometry_msgs.msg import Twist
from std_msgs.msg import String

def twist_callback(msg):
    # Serialize the Twist message into JSON format
    twist_dict = {
        'linear': {
            'x': msg.linear.x,
            'y': msg.linear.y,
            'z': msg.linear.z
        },
        'angular': {
            'x': msg.angular.x,
            'y': msg.angular.y,
            'z': msg.angular.z
        }
    }

    twist_json = json.dumps(twist_dict)

    # Publish the JSON string to /twist_string
    twist_string_pub.publish(twist_json)

def twist_to_string_node():
    rospy.init_node('twist_to_string_node', anonymous=True)

    # Subscriber to /twist
    rospy.Subscriber('/twist', Twist, twist_callback)

    # Publisher to /twist_string
    global twist_string_pub
    twist_string_pub = rospy.Publisher('/twist_string', String, queue_size=10)

    rospy.spin()

if __name__ == '__main__':
    try:
        twist_to_string_node()
    except rospy.ROSInterruptException:
        pass
