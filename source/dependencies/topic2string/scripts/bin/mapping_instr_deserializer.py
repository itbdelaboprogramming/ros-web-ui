#!/usr/bin/env python

import rospy
import json
from std_msgs.msg import Int32, String

class Int32Deserializer:
    def __init__(self):
        rospy.init_node('int32_deserializer')

        # Load parameters
        self.input_topic = rospy.get_param('~input_topic', '/string/mappingInstruc')
        self.output_topic = rospy.get_param('~output_topic', '/mapping/Instruction')

        # Publisher for deserialized data
        self.output_pub = rospy.Publisher(self.output_topic, Int32, queue_size=10)

        # Subscriber for serialized JSON
        rospy.Subscriber(self.input_topic, String, self.callback)

    def callback(self, msg):
        try:
            # Deserialize JSON string
            deserialized_data = json.loads(msg.data)

            # Create Int32 message
            int32_msg = Int32(data=deserialized_data.get("data", 0))

            # Publish deserialized data
            self.output_pub.publish(int32_msg)
            rospy.loginfo(f"Published deserialized data: {int32_msg}")

        except json.JSONDecodeError as e:
            rospy.logerr(f"Failed to decode JSON: {e}")

if __name__ == '__main__':
    try:
        Int32Deserializer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass