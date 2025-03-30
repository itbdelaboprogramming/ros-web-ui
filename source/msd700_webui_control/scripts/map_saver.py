#!/usr/bin/env python

import os
import rospy
import subprocess
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger, TriggerResponse
# Import the custom service which now accepts map_name and map_path separately
from msd700_webui_msg.srv import SetMapPath, SetMapPathResponse

class MapSaverNode:
    def __init__(self):
        # Initialize the node
        rospy.init_node('map_saver_node', anonymous=False)
        
        # Get nested parameters
        params = rospy.get_param('/map_saver', None)
        if params is None:
            rospy.logerr("Parameter '/map_saver' not loaded. Check your YAML file and launch file.")
            rospy.signal_shutdown("Missing parameters.")
            return

        # Retrieve parameters from the parameter server
        self.map_name = params.get('map_name', 'default_map')
        self.save_path = params.get('save_path', '/tmp')  # Default save directory

        self.threshold_occupied = params.get('threshold_occupied', None)
        self.threshold_free = params.get('threshold_free', None)
        
        # Log all parameters
        rospy.loginfo("Map Saver initialized with:")
        rospy.loginfo("     Map name: %s", self.map_name)
        rospy.loginfo("     Save path: %s", self.save_path)
        rospy.loginfo("     Threshold occupied: %s", self.threshold_occupied)
        rospy.loginfo("     Threshold free: %s", self.threshold_free)

        # Ensure that the save directory exists
        if not os.path.exists(self.save_path):
            os.makedirs(self.save_path)

        # Initialize a publisher for logging
        self.log_pub = rospy.Publisher('/map_saver/log', String, queue_size=10)

        # Initialize subscribers for map saver instructions
        rospy.Subscriber('/map_saver/map_instruction', Bool, self.instruction_callback)
        rospy.Subscriber('/map_saver/map_name', String, self.map_name_callback)
        rospy.Subscriber('/map_saver/save_set_map', String, self.save_set_map_callback)

        # Initialize the standard service for saving map using Trigger
        self.save_map_service = rospy.Service('/map_saver/save_map', Trigger, self.save_map_service_callback)
        # Initialize the custom service to accept separate map_name and map_path
        self.full_path_service = rospy.Service('/mapsaver/full_path', SetMapPath, self.full_path_service_callback)

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
            rospy.loginfo("MAP SAVER || Map saved successfully set to %s on %s.", self.map_name, self.save_path)
        else:
            rospy.logerr("MAP SAVER || Failed to save map %s on %s.", self.map_name, self.save_path)
        
    def save_map_service_callback(self, req):
        success = self.save_map()
        return TriggerResponse(
            success=success,
            message="Map saved successfully." if success else "Failed to save map."
        )
    
    def full_path_service_callback(self, req):
        """
        Service callback for saving the map using a provided map_name and map_path.
        The full file path is constructed as: <req.map_path>/<req.map_name>
        """
        # Construct the full file path using the provided map_path and map_name
        full_map_path = os.path.join(req.map_path, req.map_name)

        command = ['rosrun', 'map_server', 'map_saver', '-f', full_map_path]

        if self.threshold_occupied is not None:
            rospy.loginfo("MAP SAVER || Occupied threshold: %s", self.threshold_occupied)
            command.extend(['--occ', str(self.threshold_occupied)])
        if self.threshold_free is not None:
            rospy.loginfo("MAP SAVER || Free threshold: %s", self.threshold_free)
            command.extend(['--free', str(self.threshold_free)])

        try:
            # Execute the map_saver command with the provided full path
            subprocess.check_call(command)
            log_message = f"Map saved at {full_map_path}."
            rospy.loginfo(log_message)
            self.log_pub.publish(log_message)
            return SetMapPathResponse(success=True, message=log_message)
        except subprocess.CalledProcessError as e:
            log_message = f"Failed to save map: {e}"
            rospy.logerr(log_message)
            self.log_pub.publish(log_message)
            return SetMapPathResponse(success=False, message=log_message)

    def save_map(self):
        # Build the full path for the map using the current parameters
        full_map_path = os.path.join(self.save_path, self.map_name)

        command = ['rosrun', 'map_server', 'map_saver', '-f', full_map_path]

        if self.threshold_occupied is not None:
            rospy.loginfo("MAP SAVER || Occupied threshold: %s", self.threshold_occupied)
            command.extend(['--occ', str(self.threshold_occupied)])
        if self.threshold_free is not None:
            rospy.loginfo("MAP SAVER || Free threshold: %s", self.threshold_free)
            command.extend(['--free', str(self.threshold_free)])

        try:
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
