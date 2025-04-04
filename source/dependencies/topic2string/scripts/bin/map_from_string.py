#!/usr/bin/env python

import rospy
import json
import hashlib
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String


class MapFromString:
    def __init__(self):
        rospy.init_node('map_from_string')

        # Load topic names from ROS parameters
        self.map_string_topic = rospy.get_param('~map_string_topic', '/string/map')
        self.map_topic = rospy.get_param('~map_topic', '/server/slam/map')

        # Publisher for map topic
        self.map_pub = rospy.Publisher(self.map_topic, OccupancyGrid, queue_size=10)

        # Subscriber for string topic
        rospy.Subscriber(self.map_string_topic, String, self.map_string_callback)

        # Initialize hash for map comparison
        self.last_map_hash = None

    def map_string_callback(self, msg):
        try:
            # Parse JSON string to dictionary
            map_dict = json.loads(msg.data)

            # Compute hash of the map
            current_map_hash = hashlib.md5(msg.data.encode()).hexdigest()

            # Publish only if the map has changed
            if current_map_hash != self.last_map_hash:
                self.last_map_hash = current_map_hash

                # Convert dictionary to OccupancyGrid
                map_msg = OccupancyGrid()

                # Fill header
                map_msg.header.seq = map_dict["header"]["seq"]
                map_msg.header.stamp.secs = map_dict["header"]["stamp"]["secs"]
                map_msg.header.stamp.nsecs = map_dict["header"]["stamp"]["nsecs"]
                map_msg.header.frame_id = map_dict["header"]["frame_id"]

                # Fill info
                map_msg.info.map_load_time.secs = map_dict["info"]["map_load_time"]["secs"]
                map_msg.info.map_load_time.nsecs = map_dict["info"]["map_load_time"]["nsecs"]
                map_msg.info.resolution = map_dict["info"]["resolution"]
                map_msg.info.width = map_dict["info"]["width"]
                map_msg.info.height = map_dict["info"]["height"]

                # Fill origin
                map_msg.info.origin.position.x = map_dict["info"]["origin"]["position"]["x"]
                map_msg.info.origin.position.y = map_dict["info"]["origin"]["position"]["y"]
                map_msg.info.origin.position.z = map_dict["info"]["origin"]["position"]["z"]
                map_msg.info.origin.orientation.x = map_dict["info"]["origin"]["orientation"]["x"]
                map_msg.info.origin.orientation.y = map_dict["info"]["origin"]["orientation"]["y"]
                map_msg.info.origin.orientation.z = map_dict["info"]["origin"]["orientation"]["z"]
                map_msg.info.origin.orientation.w = map_dict["info"]["origin"]["orientation"]["w"]

                # Fill data
                map_msg.data = map_dict["data"]

                # Publish OccupancyGrid
                self.map_pub.publish(map_msg)
                # rospy.loginfo("Published updated OccupancyGrid.")

        except (KeyError, ValueError) as e:
            rospy.logerr(f"Error parsing map string: {e}")


if __name__ == '__main__':
    try:
        MapFromString()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
