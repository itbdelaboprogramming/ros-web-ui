#!/usr/bin/env python

import rospy
import json
from std_msgs.msg import Bool, String

class MappingSerializer:
    def __init__(self):
        rospy.init_node('mapping_serializer')

        # Load parameters
        self.start_topic = rospy.get_param('~start_topic', '/server/mapping/Start')
        self.pause_topic = rospy.get_param('~pause_topic', '/server/mapping/Pause')
        self.stop_topic = rospy.get_param('~stop_topic', '/server/mapping/Stop')
        self.output_topic = rospy.get_param('~output_topic', '/string/mappingInstruc')

        # Publisher for serialized JSON
        self.output_pub = rospy.Publisher(self.output_topic, String, queue_size=10)

        # Initialize default states
        self.mapping_data = {
            self.start_topic: False,
            self.pause_topic: False,
            self.stop_topic: False,
        }

        # Subscribers
        rospy.Subscriber(self.start_topic, Bool, self.callback, self.start_topic)
        rospy.Subscriber(self.pause_topic, Bool, self.callback, self.pause_topic)
        rospy.Subscriber(self.stop_topic, Bool, self.callback, self.stop_topic)

    def callback(self, msg, topic_name):
        
        self.mapping_data[topic_name] = msg.data
        
        for key in self.mapping_data.keys():
            if key != topic_name:
                self.mapping_data[key] = False

        # Serialize data to JSON
        serialized_data = json.dumps(self.mapping_data)

        # Publish serialized JSON string
        self.output_pub.publish(String(data=serialized_data))
        rospy.loginfo(f"Published: {serialized_data}")


if __name__ == '__main__':
    try:
        MappingSerializer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
