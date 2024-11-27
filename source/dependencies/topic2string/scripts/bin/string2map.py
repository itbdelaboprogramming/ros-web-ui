#!/usr/bin/env python3

import rospy
import json
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String

def string_callback(msg):
    # Deserialize the JSON string into a Python dictionary
    map_dict = json.loads(msg.data)

    # Convert the dictionary back to an OccupancyGrid message
    map_msg = OccupancyGrid()
    map_msg.info.resolution = map_dict['info']['resolution']
    map_msg.info.width = map_dict['info']['width']
    map_msg.info.height = map_dict['info']['height']
    map_msg.info.origin.position.x = map_dict['info']['origin']['position']['x']
    map_msg.info.origin.position.y = map_dict['info']['origin']['position']['y']
    map_msg.info.origin.position.z = map_dict['info']['origin']['position']['z']
    map_msg.info.origin.orientation.x = map_dict['info']['origin']['orientation']['x']
    map_msg.info.origin.orientation.y = map_dict['info']['origin']['orientation']['y']
    map_msg.info.origin.orientation.z = map_dict['info']['origin']['orientation']['z']
    map_msg.info.origin.orientation.w = map_dict['info']['origin']['orientation']['w']
    map_msg.data = map_dict['data']

    # Publish the decoded OccupancyGrid message
    map_decoded_pub.publish(map_msg)

def string_to_map_node():
    rospy.init_node('string_to_map_node')

    # Subscriber to /map_string
    rospy.Subscriber('/map_string', String, string_callback)

    # Publisher to /map_decoded
    global map_decoded_pub
    map_decoded_pub = rospy.Publisher('/map_decoded', OccupancyGrid, queue_size=10)

    rospy.spin()

if __name__ == '__main__':
    try:
        string_to_map_node()
    except rospy.ROSInterruptException:
        pass
