#!/usr/bin/env python3
import rospy
import json
from std_msgs.msg import String
from msd700_webui_msg.msg import SwitchModeMsg
from msd700_webui_msg.srv import SwitchMode

class ChangeModeServiceNode:
    def __init__(self):
        rospy.init_node('change_mode_service_node', anonymous=False)
        
        self.input_topic = rospy.get_param('~input_topic', '/string/switch_mode')
        self.service_name = rospy.get_param('~service_name', '/switch_mode')
        
        self.sub = rospy.Subscriber(self.input_topic, String, self.callback)
        
        # Give log detailed info
        rospy.loginfo("Change Mode Deserialization Node is active")
        rospy.loginfo("     Subscribe to: %s", self.input_topic)
        rospy.loginfo("     Service name: %s", self.service_name)
        
        # tmeout service
        rospy.wait_for_service(self.service_name, timeout=10.0)
        self.switch_mode_srv = rospy.ServiceProxy(self.service_name, SwitchMode)
        
        rospy.loginfo("ChangeModeServiceNode activated.")
    
    def callback(self, msg):
        try:
            # Deserialisasi pesan JSON menjadi dictionary
            data = json.loads(msg.data)
            rospy.loginfo("Data JSON diterima: %s", data)
        except json.JSONDecodeError as e:
            rospy.logerr("Gagal mendeserialisasi JSON: %s", e)
            return
        
        # Konversi dictionary menjadi objek SwitchModeMsg
        switch_mode_msg = SwitchModeMsg()
        switch_mode_msg.mode = data.get("mode", "")
        switch_mode_msg.open_rviz = data.get("open_rviz", False)
        switch_mode_msg.use_simulator = data.get("use_simulator", False)
        switch_mode_msg.map_file = data.get("map_file", "")
        switch_mode_msg.point_mode = data.get("point_mode", "")
        
        # Panggil service
        try:
            response = self.switch_mode_srv(switch_mode_msg)
            rospy.loginfo("Respon service: success=%s, message=%s", 
                          response.success, response.message)
        except rospy.ServiceException as e:
            rospy.logerr("Pemanggilan service gagal: %s", e)

if __name__ == '__main__':
    try:
        node = ChangeModeServiceNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
