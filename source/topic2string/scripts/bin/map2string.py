#!/usr/bin/env python3

import rospy
import json
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String

def map_callback(msg):
    # Serialize the OccupancyGrid message into a JSON format
    map_dict = {
        'info': {
            'resolution': msg.info.resolution,
            'width': msg.info.width,
            'height': msg.info.height,
            'origin': {
                'position': {
                    'x': msg.info.origin.position.x,
                    'y': msg.info.origin.position.y,
                    'z': msg.info.origin.position.z
                },
                'orientation': {
                    'x': msg.info.origin.orientation.x,
                    'y': msg.info.origin.orientation.y,
                    'z': msg.info.origin.orientation.z,
                    'w': msg.info.origin.orientation.w
                }
            }
        },
        'data': msg.data  # List of occupancy values
    }

    map_json = json.dumps(map_dict)

    # Publish the JSON string to /map_string
    map_string_pub.publish(map_json)

def map_to_string_node():
    rospy.init_node('map_to_string_node')

    # Subscriber to /map
    rospy.Subscriber('/map', OccupancyGrid, map_callback)

    # Publisher to /map_string
    global map_string_pub
    map_string_pub = rospy.Publisher('/map_string', String, queue_size=10)

    rospy.spin()

if __name__ == '__main__':
    try:
        map_to_string_node()
    except rospy.ROSInterruptException:
        pass
