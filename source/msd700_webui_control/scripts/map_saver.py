#!/usr/bin/env python

import os
import rospy
import subprocess
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger, TriggerResponse

class MapSaverNode:
    def __init__(self):
        # Inisialisasi node
        rospy.init_node('map_saver_node', anonymous=False)
        
        # Get nested parameters
        params = rospy.get_param('/map_saver', None)
        if params is None:
            rospy.logerr("Parameter '/map_saver' not loaded. Check your YAML file and launch file.")
            rospy.signal_shutdown("Missing parameters.")
            return

        # Mendapatkan parameter dari server parameter
        self.map_name = params.get('map_name', 'default_map')
        self.save_path = params.get('save_path', '/tmp')  # Direktori penyimpanan

        if rospy.has_param('threshold_occupied'):
            self.threshold_occupied = params.get('threshold_occupied')
        else:
            self.threshold_occupied = None

        # Memeriksa dan mendapatkan parameter threshold_free
        if rospy.has_param('threshold_free'):
            self.threshold_free = rospy.get_param('threshold_free')
        else:
            self.threshold_free = None
        
        # Print all parameters
        rospy.loginfo("Map Saver initialized with:")
        rospy.loginfo("  Map name: %s", self.map_name)
        rospy.loginfo("  Save path: %s", self.save_path)
        rospy.loginfo("  Threshold occupied: %s", self.threshold_occupied)
        rospy.loginfo("  Threshold free: %s", self.threshold_free)

        # Pastikan direktori penyimpanan ada
        if not os.path.exists(self.save_path):
            os.makedirs(self.save_path)

        # Inisialisasi publisher untuk log
        self.log_pub = rospy.Publisher('/map_saver/log', String, queue_size=10)

        # Inisialisasi subscriber untuk instruksi penyimpanan peta
        rospy.Subscriber('/map_saver/map_instruction', Bool, self.instruction_callback)
        rospy.Subscriber('/map_saver/map_name', String, self.map_name_callback)
        rospy.Subscriber('/map_saver/save_set_map', String, self.save_set_map_callback)

        # Inisialisasi service untuk penyimpanan peta
        self.save_map_service = rospy.Service('/map_saver/save_map', Trigger, self.save_map_service_callback)

        rospy.loginfo("Map Saver Node initialized.")
        self.log_pub.publish("Map Saver Node initialized.")

    def instruction_callback(self, msg):
        if msg.data:
            self.save_map()
    
    def map_name_callback(self, msg):
        self.map_name = msg.data
        
    def save_set_map_callback(self, msg):
        self.map_name = msg.data
        success = self.save_map()
        if success:
            rospy.loginfo("Map saved successfully set to %s on %s.", self.map_name, self.save_path)
        else:
            rospy.logerr("Failed to save map %s on %s.", self.map_name, self.save_path)
        
    def save_map_service_callback(self, req):
        success = self.save_map()
        return TriggerResponse(
            success=success,
            message="Map saved successfully." if success else "Failed to save map."
        )

    def save_map(self):
        # Membangun path lengkap untuk file peta
        full_map_path = os.path.join(self.save_path, self.map_name)

        # Membangun perintah untuk menjalankan map_saver
        command = ['rosrun', 'map_server', 'map_saver', '-f', full_map_path]

        if self.threshold_occupied is not None:
            rospy.loginfo("Occupied threshold: %s", self.threshold_occupied)
            command.extend(['--occ', str(self.threshold_occupied)])
        if self.threshold_free is not None:
            rospy.loginfo("Free threshold: %s", self.threshold_free)
            command.extend(['--free', str(self.threshold_free)])

        try:
            # Menjalankan perintah map_saver
            subprocess.check_call(command)
            log_message = f"Map saved at {full_map_path}."
            rospy.loginfo(log_message)
            self.log_pub.publish(log_message)
            return True
        except subprocess.CalledProcessError as e:
            log_message = f"Failed to save map: {e}"
            rospy.logerr(log_message)
            self.log_pub.publish(log_message)
            return False

if __name__ == '__main__':
    try:
        MapSaverNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Map Saver Node terminated.")