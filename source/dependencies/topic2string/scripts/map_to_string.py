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
        self.publish_rate = rospy.get_param('~publish_rate', 0.5)  # Publish rate in Hz
        self.debug = rospy.get_param('~debug', False)

        # Publisher for string topic
        self.map_string_pub = rospy.Publisher(self.map_string_topic, String, queue_size=10)

        # Initialize variables
        self.latest_map_msg = None
        self.last_map_hash = None

        # Subscriber for map topic
        rospy.Subscriber(self.map_topic, OccupancyGrid, self.map_callback)

        # Timer for periodic publishing
        rospy.Timer(rospy.Duration(1.0 / self.publish_rate), self.publish_latest_map)

        rospy.loginfo(f"Listening to {self.map_topic} and publishing to {self.map_string_topic} at {self.publish_rate} Hz")

    def map_callback(self, map_msg):
        """Callback function to store the latest map message."""
        self.latest_map_msg = map_msg

    def publish_latest_map(self, event):
        """Publish the latest map as a JSON string at a fixed interval."""
        if self.latest_map_msg is None:
            return

        try:
            # Convert OccupancyGrid to dictionary
            map_dict = {
                "header": {
                    "seq": self.latest_map_msg.header.seq,
                    "stamp": {
                        "secs": self.latest_map_msg.header.stamp.secs,
                        "nsecs": self.latest_map_msg.header.stamp.nsecs,
                    },
                    "frame_id": self.latest_map_msg.header.frame_id,
                },
                "info": {
                    "map_load_time": {
                        "secs": self.latest_map_msg.info.map_load_time.secs,
                        "nsecs": self.latest_map_msg.info.map_load_time.nsecs,
                    },
                    "resolution": self.latest_map_msg.info.resolution,
                    "width": self.latest_map_msg.info.width,
                    "height": self.latest_map_msg.info.height,
                    "origin": {
                        "position": {
                            "x": self.latest_map_msg.info.origin.position.x,
                            "y": self.latest_map_msg.info.origin.position.y,
                            "z": self.latest_map_msg.info.origin.position.z,
                        },
                        "orientation": {
                            "x": self.latest_map_msg.info.origin.orientation.x,
                            "y": self.latest_map_msg.info.origin.orientation.y,
                            "z": self.latest_map_msg.info.origin.orientation.z,
                            "w": self.latest_map_msg.info.origin.orientation.w,
                        },
                    },
                },
                "data": self.latest_map_msg.data,  # OccupancyGrid data array
            }

            # Compute hash of the map
            current_map_hash = hashlib.md5(json.dumps(map_dict, sort_keys=True).encode()).hexdigest()

            self.last_map_hash = current_map_hash
            map_string = json.dumps(map_dict)
            self.map_string_pub.publish(map_string)
            if self.debug:
                rospy.loginfo("Published updated map string.")

            # # Publish only if the map has changed
            # if current_map_hash != self.last_map_hash:
            #     self.last_map_hash = current_map_hash
            #     map_string = json.dumps(map_dict)
            #     self.map_string_pub.publish(map_string)
            #     rospy.loginfo("Published updated map string.")

        except Exception as e:
            rospy.logerr(f"Error processing map data: {e}")


if __name__ == '__main__':
    try:
        MapToString()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
