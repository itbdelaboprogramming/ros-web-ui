#!/usr/bin/env python

import rospy
import json
from nav_msgs.srv import GetMap
from std_msgs.msg import String


class MapToStringService:
    def __init__(self):
        rospy.init_node('map_to_string_service')

        # Load parameters
        self.map_string_topic = rospy.get_param('~map_string_topic', '/string/map')
        self.static_map_service = rospy.get_param('~static_map_service', '/static_map')
        self.publish_frequency = rospy.get_param('~publish_frequency', 100.0)  # Default 2 Hz

        # Publisher for string topic
        self.map_string_pub = rospy.Publisher(self.map_string_topic, String, queue_size=10)

        # Wait for the service to be available
        rospy.loginfo(f"Waiting for service {self.static_map_service}...")
        rospy.wait_for_service(self.static_map_service)
        self.get_map_service = rospy.ServiceProxy(self.static_map_service, GetMap)
        rospy.loginfo(f"Connected to service {self.static_map_service}")

        # Set up periodic publishing
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.publish_frequency), self.timer_callback)

    def timer_callback(self, event):
        try:
            # Call the service
            response = self.get_map_service()
            map_msg = response.map

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

            # Convert dictionary to JSON string
            map_string = json.dumps(map_dict)

            # Publish JSON string
            rospy.loginfo(f"Publishing map string to {self.map_string_topic}")
            self.map_string_pub.publish(map_string)

        except rospy.ServiceException as e:
            rospy.logerr(f"Service call to {self.static_map_service} failed: {e}")


if __name__ == '__main__':
    try:
        MapToStringService()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
