#!/usr/bin/env python

import rospy
import json
from std_msgs.msg import Int32, String

class Int32Serializer:
    def __init__(self):
        rospy.init_node('int32_serializer')

        # Load parameters
        self.input_topic = rospy.get_param('~input_topic', '/server/mapping/Instruction')
        self.output_topic = rospy.get_param('~output_topic', '/string/mappingInstruc')

        # Publisher for serialized JSON
        self.output_pub = rospy.Publisher(self.output_topic, String, queue_size=10)

        # Subscriber
        rospy.Subscriber(self.input_topic, Int32, self.callback)

    def callback(self, msg):
        # Serialize the Int32 message to JSON
        serialized_data = json.dumps({"data": msg.data})

        # Publish serialized JSON string
        self.output_pub.publish(String(data=serialized_data))
        rospy.loginfo(f"Published serialized data: {serialized_data}")

if __name__ == '__main__':
    try:
        Int32Serializer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass