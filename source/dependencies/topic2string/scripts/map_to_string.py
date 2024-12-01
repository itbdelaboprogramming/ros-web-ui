#!/usr/bin/env python

import rospy
import json
import hashlib
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String


class MapToString:
    def __init__(self):
        rospy.init_node('map_to_string')

        # Load parameters
        self.map_string_topic = rospy.get_param('~map_string_topic', '/string/map')
        self.map_topic = rospy.get_param('~map_topic', '/map')

        # Publisher for string topic
        self.map_string_pub = rospy.Publisher(self.map_string_topic, String, queue_size=10)

        # Subscriber for map topic
        rospy.Subscriber(self.map_topic, OccupancyGrid, self.map_callback)

        # Initialize hash for map comparison
        self.last_map_hash = None
        rospy.loginfo(f"Listening to {self.map_topic} and publishing changes to {self.map_string_topic}")

    def map_callback(self, map_msg):
        try:
            # Convert OccupancyGrid to dictionary
            map_dict = {
                "header": {
                    "seq": map_msg.header.seq,
                    "stamp": {"secs": map_msg.header.stamp.secs, "nsecs": map_msg.header.stamp.nsecs},
                    "frame_id": map_msg.header.frame_id,
                },
                "info": {
                    "map_load_time": {"secs": map_msg.info.map_load_time.secs, "nsecs": map_msg.info.map_load_time.nsecs},
                    "resolution": map_msg.info.resolution,
                    "width": map_msg.info.width,
                    "height": map_msg.info.height,
                    "origin": {
                        "position": {
                            "x": map_msg.info.origin.position.x,
                            "y": map_msg.info.origin.position.y,
                            "z": map_msg.info.origin.position.z,
                        },
                        "orientation": {
                            "x": map_msg.info.origin.orientation.x,
                            "y": map_msg.info.origin.orientation.y,
                            "z": map_msg.info.origin.orientation.z,
                            "w": map_msg.info.origin.orientation.w,
                        },
                    },
                },
                "data": map_msg.data,  # OccupancyGrid data array
            }

            # Compute hash of the map
            current_map_hash = hashlib.md5(json.dumps(map_dict, sort_keys=True).encode()).hexdigest()

            # Publish only if the map has changed
            if current_map_hash != self.last_map_hash:
                self.last_map_hash = current_map_hash
                map_string = json.dumps(map_dict)
                self.map_string_pub.publish(map_string)
                rospy.loginfo("Published updated map string.")

        except Exception as e:
            rospy.logerr(f"Error processing map data: {e}")


if __name__ == '__main__':
    try:
        MapToString()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
