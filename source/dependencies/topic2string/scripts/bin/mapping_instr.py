#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32, Bool

class Int32ToBoolTopics:
    def __init__(self):
        rospy.init_node('int32_to_bool_topics')

        # Load parameters
        self.input_topic = rospy.get_param('~input_topic', '/mapping/Instruction')
        self.start_topic = rospy.get_param('~start_topic', '/mapping/start')
        self.pause_topic = rospy.get_param('~pause_topic', '/mapping/Pause')
        self.stop_topic = rospy.get_param('~stop_topic', '/mapping/stop')

        # Publishers for Bool messages
        self.start_pub = rospy.Publisher(self.start_topic, Bool, queue_size=10)
        self.pause_pub = rospy.Publisher(self.pause_topic, Bool, queue_size=10)
        self.stop_pub = rospy.Publisher(self.stop_topic, Bool, queue_size=10)

        # Subscriber to Int32 topic
        rospy.Subscriber(self.input_topic, Int32, self.callback)

    def callback(self, msg):
        # Publish Bool messages based on Int32 data
        # start_msg = Bool(data=(msg.data == 1))
        # pause_msg = Bool(data=(msg.data == 2))
        # stop_msg = Bool(data=(msg.data == 3))

        # self.start_pub.publish(start_msg)
        # self.pause_pub.publish(pause_msg)
        # self.stop_pub.publish(stop_msg)

        # rospy.loginfo(f"Published to {self.start_topic}: {start_msg}")
        # rospy.loginfo(f"Published to {self.pause_topic}: {pause_msg}")
        # rospy.loginfo(f"Published to {self.stop_topic}: {stop_msg}")
        
        if(msg.data == 1):
            start_msg = Bool(data=True)
            pause_msg = Bool(data=False)
            self.start_pub.publish(start_msg)
            self.pause_pub.publish(pause_msg)
            rospy.loginfo(f"Published to {self.start_topic}: {start_msg}")
        elif msg.data == 2:
            start_msg = Bool(data=False)
            pause_msg = Bool(data=True)
            self.start_pub.publish(start_msg)
            self.pause_pub.publish(pause_msg)
            rospy.loginfo(f"Published to {self.pause_topic}: {pause_msg}")
        elif msg.data == 3:
            stop_msg = Bool(data=True)
            self.stop_pub.publish(stop_msg)
            rospy.loginfo(f"Published to {self.stop_topic}: {stop_msg}")
        else:
            start_msg = Bool(data=False)
            pause_msg = Bool(data=False)
            stop_msg = Bool(data=False)
            
            self.start_pub.publish(start_msg)
            self.pause_pub.publish(pause_msg)
            self.stop_pub.publish(stop_msg)

if __name__ == '__main__':
    try:
        Int32ToBoolTopics()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass