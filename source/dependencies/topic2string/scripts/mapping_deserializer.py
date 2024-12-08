#!/usr/bin/env python

import rospy
import json
from std_msgs.msg import Bool, String

class MappingDeserializer:
    def __init__(self):
        rospy.init_node('mapping_deserializer')

        # Load parameters
        self.input_topic = rospy.get_param('~input_topic', '/string/mappingInstruc')
        self.start_topic = rospy.get_param('~start_topic', '/mapping/Start')
        self.pause_topic = rospy.get_param('~pause_topic', '/mapping/Pause')
        self.stop_topic = rospy.get_param('~stop_topic', '/mapping/Stop')

        # Publishers for deserialized topics
        self.start_pub = rospy.Publisher(self.start_topic, Bool, queue_size=10)
        self.pause_pub = rospy.Publisher(self.pause_topic, Bool, queue_size=10)
        self.stop_pub = rospy.Publisher(self.stop_topic, Bool, queue_size=10)

        # Subscriber for serialized JSON
        rospy.Subscriber(self.input_topic, String, self.callback)

    def callback(self, msg):
        try:
            # Deserialize JSON string
            mapping_data = json.loads(msg.data)

            # Publish to respective topics
            for topic_name, value in mapping_data.items():
                if topic_name == self.start_topic:
                    self.start_pub.publish(Bool(data=value))
                elif topic_name == self.pause_topic:
                    self.pause_pub.publish(Bool(data=value))
                elif topic_name == self.stop_topic:
                    self.stop_pub.publish(Bool(data=value))

            rospy.loginfo(f"Deserialized and published: {mapping_data}")

        except json.JSONDecodeError as e:
            rospy.logerr(f"Failed to decode JSON: {e}")



if __name__ == '__main__':
    try:
        MappingDeserializer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
