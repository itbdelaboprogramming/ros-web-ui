#!/usr/bin/env python3
import rospy
import json
from msd700_webui_msg.msg import SwitchModeMsg
from std_msgs.msg import String

class ChangeModeSerializeNode:
    def __init__(self):
        rospy.init_node('change_mode_serialize_node', anonymous=False)
        
        self.input_topic = rospy.get_param('~input_topic', '/server/change_mode')
        self.output_topic = rospy.get_param('~output_topic', '/string/change_mode')
        
        # Buat publisher dan subscriber
        self.pub = rospy.Publisher(self.output_topic, String, queue_size=10)
        self.sub = rospy.Subscriber(self.input_topic, SwitchModeMsg, self.callback)
        
        rospy.loginfo("ChangeModeSerializeNode aktif. Subscribe: %s, Publish: %s", 
                      self.input_topic, self.output_topic)
    
    def callback(self, msg):
        # Konversi pesan SwitchModeMsg ke dictionary
        data = {
            "mode": msg.mode,
            "open_rviz": msg.open_rviz,
            "use_simulator": msg.use_simulator,
            "map_file": msg.map_file,
            "point_mode": msg.point_mode
        }
        # Serialisasi ke string JSON
        json_str = json.dumps(data)

        self.pub.publish(json_str)
        rospy.loginfo("JSON dikirim: %s", json_str)

if __name__ == '__main__':
    try:
        node = ChangeModeSerializeNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
