#!/usr/bin/env python3
import os
import subprocess
import json
import time
from abc import ABC, abstractmethod

from msd700_webui_msg.srv import SwitchMode, SwitchModeResponse
from msd700_webui_msg.msg import SwitchModeMsg
from msd700_webui_msg.srv import SetMapPath, SetMapPathResponse

import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool

def wait_for_file(file_path, timeout=30, check_interval=0.5):
    """Wait until the file exists and its size has stabilized."""
    rospy.loginfo(f"Waiting for file {file_path} to be created...")
    start_time = time.time()
    previous_size = -1

    while True:
        if os.path.exists(file_path):
            current_size = os.path.getsize(file_path)
            # If file size has stabilized (i.e., no change between checks), assume file is complete
            if current_size == previous_size:
                rospy.loginfo("File creation complete and stable.")
                break
            previous_size = current_size

        if time.time() - start_time > timeout:
            rospy.logerr("Timeout waiting for file to be created or stabilized.")
            break

        rospy.sleep(check_interval)

class CommandHandler(ABC):
    """Abstract base class untuk command handlers"""

    @abstractmethod
    def can_handle(self, header: str) -> bool:
        pass
    
    @abstractmethod
    def handle_command(self, command_data: dict):
        pass

class MappingCommandHandler(CommandHandler):
    """Handler for mapping commands"""

    def __init__(self):
        super().__init__()
        
        # ======== Initialization ========
        # Initialize the service proxy for /switch_mode
        self.switch_mode_srv = rospy.ServiceProxy('/switch_mode', SwitchMode)
        self.save_map_srv = rospy.ServiceProxy('/mapsaver/full_path', SetMapPath)
        # Initialize the topic publisher for pause command
        self.pause_pub = rospy.Publisher('/emergency_pause', Bool, queue_size=10, latch=True)
        # ======== Variable Mappings ========
        self.is_mapping = False
        
        self.actions = {
            'start': self._start_mapping,
            'pause': self._pause_mapping,
            'stop': self._stop_mapping
        }
    
    def can_handle(self, header: str) -> bool:
        return header == "mapping"
    
    def handle_command(self, command_data: dict):
        command = command_data.get('command')
        action = self.actions.get(command)
        
        if action:
            action(command_data)
        else:
            rospy.logwarn(f"Unknown mapping command: {command}")

    def _start_mapping(self, data: dict):
        rospy.loginfo("Starting mapping process...")
        # Create and publish the pause state message
        pause_msg = Bool(data=False)
        self.pause_pub.publish(pause_msg)
        
        # Prepare the service request
        mode_msg = SwitchModeMsg(
            mode='explore',
            open_rviz=True,
            use_simulator=True,
            map_file='NaN',
            point_mode='NaN'
        )
        
        if not self.is_mapping:
            try:
                rospy.loginfo(f"Calling /switch_mode service with: {mode_msg}")
                response = self.switch_mode_srv(mode_msg)
                self.is_mapping = True
                rospy.loginfo(f"Service response: success={response.success}, message={response.message}")
            except rospy.ServiceException as e:
                self.is_mapping = False
                rospy.logerr(f"Service call to /switch_mode failed: {str(e)}")
        else:
            rospy.loginfo("Already in mapping mode.")
            
        # Wait for 2 seconds before sending the switch mode command
        rospy.loginfo("Waiting 2 seconds before switching mode...")
        time.sleep(1)

    def _pause_mapping(self, data: dict):
        rospy.loginfo("Pausing mapping process...")
        # Create and publish the pause state message
        pause_msg = Bool(data=True)
        self.pause_pub.publish(pause_msg)
        rospy.loginfo("Published 'True' to /emergency_pause topic")

    def _stop_mapping(self, data: dict):
        rospy.loginfo("Stopping mapping process...")
        
        config = data.get('config', {})
        map_name = config.get('resource', {}).get('map_name', 'default_map')
        save_path = config.get('resource', {}).get('default_save_path', '/default/path')
        
        rospy.loginfo(f"Saving map {map_name} to {save_path}")
        # Construct the map path as save_path + '/msd'
        map_path = f"{save_path}/msd"
        
        try:
            rospy.loginfo(f"Calling /map_saver/save_map service with: map_path={map_path}, map_name={map_name}")
            # Pass the arguments directly to the service proxy
            response = self.save_map_srv(map_name, map_path)
            rospy.loginfo(f"Service response: success={response.success}, message={response.message}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call to /map_saver/save_map failed: {str(e)}")
        
        # Create and publish the pause state message
        pause_msg = Bool(data=True)
        self.pause_pub.publish(pause_msg)

        # Define the expected output file, e.g., a .pgm file created by map_saver
        expected_file = f"{map_path}/{map_name}.pgm"
        wait_for_file(expected_file, timeout=30, check_interval=0.5)
        
        # Prepare the service request for switching to idle mode
        mode_msg = SwitchModeMsg(
            mode='idle',
            open_rviz=False,
            use_simulator=False,
            map_file='NaN',
            point_mode='NaN'
        )
        
        try:
            rospy.loginfo(f"Calling /switch_mode service with: {mode_msg}")
            response = self.switch_mode_srv(mode_msg)
            rospy.loginfo(f"Service response: success={response.success}, message={response.message}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call to /switch_mode failed: {str(e)}")


class NavigationCommandHandler(CommandHandler):
    """Handler untuk command navigation"""
    
    def __init__(self):
        self.actions = {
            'init': self._init_navigation,
            'start': self._start_navigation,
            'pause': self._pause_navigation
        }
    
    def can_handle(self, header: str) -> bool:
        return header == "navigation"
    
    def handle_command(self, command_data: dict):
        command = command_data.get('command')
        action = self.actions.get(command)
        
        if action:
            action(command_data)
        else:
            rospy.logwarn(f"Unknown navigation command: {command}")

    def _init_navigation(self, data: dict):
        config = data.get('config', {})
        map_file = config.get('resource', {}).get('map_file', '')
        operation_mode = config.get('operation_mode', 'single')
        
        rospy.loginfo(f"Initializing {operation_mode} mode navigation with map: {map_file}")
        
    def _start_navigation(self, data: dict):
        rospy.loginfo("Starting navigation...")
        targets = data.get('data', {}).get('targets', [])
        for target in targets:
            rospy.loginfo(f"Navigating to: {target}")
        # Implementasi start navigation
        
    def _pause_navigation(self, data: dict):
        rospy.loginfo("Pausing navigation...")
        # Implementasi pause navigation

class SystemCommandNode:
    def __init__(self):
        rospy.init_node('system_command_node')
        
        # Daftar handler yang tersedia
        self.handlers = [
            MappingCommandHandler(),
            NavigationCommandHandler()
        ]
        
        # Subscriber ke topic SystemCommand
        self.sub = rospy.Subscriber(
            '/system_command', 
            String, 
            self.command_callback,
            queue_size=10
        )
        
        rospy.loginfo("System Command Node initialized")

    def command_callback(self, msg):
        try:
            command_data = json.loads(msg.data)
            self.process_command(command_data)
        except json.JSONDecodeError:
            rospy.logerr("Received invalid JSON command")
            rospy.logerr(f"Command data: {msg.data}")
        except Exception as e:
            rospy.logerr(f"Error processing command: {str(e)}")

    def process_command(self, command_data: dict):
        header = command_data.get('header')
        if not header:
            rospy.logwarn("Received command without header")
            return
            
        for handler in self.handlers:
            if handler.can_handle(header):
                try:
                    handler.handle_command(command_data)
                    return
                except Exception as e:
                    rospy.logerr(f"Error executing {header} command: {str(e)}")
                    return
        
        rospy.logwarn(f"No handler found for header: {header}")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = SystemCommandNode()
        node.run()
    except rospy.ROSInterruptException:
        pass