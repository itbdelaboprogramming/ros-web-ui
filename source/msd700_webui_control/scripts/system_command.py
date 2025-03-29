#!/usr/bin/env python3
import rospy
import json
from abc import ABC, abstractmethod
from std_msgs.msg import String

class CommandHandler(ABC):
    """Abstract base class untuk command handlers"""

    @abstractmethod
    def can_handle(self, header: str) -> bool:
        pass
    
    @abstractmethod
    def handle_command(self, command_data: dict):
        pass

class MappingCommandHandler(CommandHandler):
    """Handler untuk command mapping"""
    
    def __init__(self):
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
        # Implementasi start mapping
        # Contoh: 
        # 1. Initiate slam toolbox
        # 2. Start data acquisition
        
    def _pause_mapping(self, data: dict):
        rospy.loginfo("Pausing mapping process...")
        # Implementasi pause logic
        
    def _stop_mapping(self, data: dict):
        rospy.loginfo("Stopping mapping process...")
        config = data.get('config', {})
        map_name = config.get('resource', {}).get('map_name', 'default_map')
        save_path = config.get('resource', {}).get('default_save_path', '/default/path')
        
        rospy.loginfo(f"Saving map {map_name} to {save_path}")
        # Implementasi save map logic

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