#!/usr/bin/env python

import rospy
import json
from std_msgs.msg import Bool, String

class MappingDeserializer:
    def __init__(self):
        rospy.init_node('mapping_deserializer')

        # Load parameters
        self.input_topic = rospy.get_param('~input_topic', '/string/mappingInstruc')
        self.pause_topic = rospy.get_param('~pause_topic', '/mapping/Pause')
        self.stop_topic = rospy.get_param('~stop_topic', '/mapping/Stop')

        # Publishers for deserialized topics
        self.pause_pub = rospy.Publisher(self.pause_topic, Bool, queue_size=10)
        self.stop_pub = rospy.Publisher(self.stop_topic, Bool, queue_size=10)

        # Subscriber for serialized JSON
        rospy.Subscriber(self.input_topic, String, self.callback)

    def callback(self, msg):
        try:
            # Deserialize JSON string
            mapping_data = json.loads(msg.data)

            for topic_name, value in mapping_data.items():
                topic_name = topic_name.replace("/server", "")  # Remove prefix
                bool_msg = Bool(data=value)

                if topic_name == self.pause_topic:
                    self.pause_pub.publish(bool_msg)
                    rospy.loginfo(f"Published to {self.pause_topic}: {bool_msg}")
                elif topic_name == self.stop_topic:
                    self.stop_pub.publish(bool_msg)
                    rospy.loginfo(f"Published to {self.stop_topic}: {bool_msg}")


        except json.JSONDecodeError as e:
            rospy.logerr(f"Failed to decode JSON: {e}")


if __name__ == '__main__':
    try:
        MappingDeserializer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass