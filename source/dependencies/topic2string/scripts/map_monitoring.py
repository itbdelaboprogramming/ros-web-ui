#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

class ByteSizeMonitor:
    """
    ROS Node to monitor the data size of a string topic
    """
    
    def __init__(self):
        """
        Initialize the node and parameters
        """
        rospy.init_node('byte_size_monitor', anonymous=False)
        
        # Ambil parameter dari server parameter
        self.topic_name = rospy.get_param('~topic_name', '/string/map')
        self.queue_size = rospy.get_param('~queue_size', 10)
        self.print_to_console = rospy.get_param('~print_to_console', True)
        self.log_level = rospy.get_param('~log_level', 'INFO').upper()
        
        # Setup subscriber
        self.sub = rospy.Subscriber(
            self.topic_name,
            String,
            self.callback,
            queue_size=self.queue_size
        )
        
        # Setup logger level
        self.logger = self._setup_logger()
        
        rospy.loginfo("Byte Size Monitor initialized for topic: %s", self.topic_name)
        rospy.logdebug("Configuration:\nQueue Size: %d\nPrint to Console: %s\nLog Level: %s",
                       self.queue_size, self.print_to_console, self.log_level)
    
    def _setup_logger(self):
        """
        Returns the logger function corresponding to the specified level
        """
        log_levels = {
            'DEBUG': rospy.logdebug,
            'INFO': rospy.loginfo,
            'WARN': rospy.logwarn,
            'ERROR': rospy.logerr,
            'FATAL': rospy.logfatal
        }
        return log_levels.get(self.log_level, rospy.loginfo)
    
    def callback(self, msg):
        """
        Callback to process incoming messages
        """
        try:
            # Hitung ukuran byte
            byte_size = len(msg.data.encode('utf-8'))
            
            # Format output
            log_message = f"Received message size: {byte_size} bytes"
            
            # Tampilkan sesuai konfigurasi
            if self.print_to_console:
                print(log_message)
            
            self.logger(log_message)
            
        except Exception as e:
            rospy.logerr("Error processing message: %s", str(e))
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        monitor = ByteSizeMonitor()
        monitor.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down Byte Size Monitor node")