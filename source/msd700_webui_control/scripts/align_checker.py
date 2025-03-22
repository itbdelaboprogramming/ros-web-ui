#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import Bool

class RobotAlignmentController:
    def __init__(self):
        rospy.init_node('robot_alignment_controller', anonymous=False)
        
        # Get nested parameters
        params = rospy.get_param('/align_control_params', None)
        if params is None:
            rospy.logerr("Parameter '/align_control_params' not loaded. Check your YAML file and launch file.")
            rospy.signal_shutdown("Missing parameters.")
            return

        # Load parameters
        self.check_frequency = params.get('check_frequency', 1.0)  # Frequency in Hz
        self.timeout = params.get('timeout', 30.0)  # Timeout in seconds
        self.rotation_speed = params.get('rotation_speed', 0.5)  # Rotation speed for z-axis
        self.alignment_topic = params.get('alignment_topic', '/check_alignment')
        self.cmd_vel_topic = params.get('cmd_vel_topic', '/mux/allign')
        self.service_start = params.get('service_start', '/alignment/start')
        self.service_reset = params.get('service_reset', '/alignment/reset')
        self.service_allign_status = params.get('service_allign_status', '/client/allign')

        # State variables
        self.aligned = False
        self.rotation_active = False
        self.last_attempt_time = rospy.Time.now()
        self.is_active = False  # Added state to control active status

        # Publishers and Services
        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=10)
        rospy.Service(self.service_reset, Trigger, self.reset_alignment_service)
        rospy.Service(self.service_start, Trigger, self.start_alignment_service)

        # Subscriber to alignment status
        rospy.Subscriber(self.service_allign_status, Bool, self.alignment_status_callback)

        # Timer for checking alignment
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.check_frequency), self.check_alignment)

        rospy.loginfo("Robot Alignment Controller initialized with:")
        rospy.loginfo("     Check frequency: %f Hz", self.check_frequency)
        rospy.loginfo("     Timeout: %f seconds", self.timeout)
        rospy.loginfo("     Rotation speed: %f", self.rotation_speed)
        rospy.loginfo("     Alignment topic: %s", self.alignment_topic)
        rospy.loginfo("     Command velocity topic: %s", self.cmd_vel_topic)
        rospy.loginfo("     Service start: %s", self.service_start)
        rospy.loginfo("     Service reset: %s", self.service_reset)
        rospy.loginfo("     Service alignment status: %s", self.service_allign_status)
        

    def alignment_status_callback(self, msg):
        self.aligned = msg.data
        rospy.loginfo("ALLIGN CHECK || Alignment status updated: %s", self.aligned)

    def check_alignment(self, event):
        if not self.is_active:
            # rospy.loginfo_throttle(5, "ALLIGN CHECK || Waiting for start command.")
            return

        if self.aligned:
            rospy.loginfo("ALLIGN CHECK || Robot already aligned. No action required.")
            self.stop_rotation()
            return

        if (rospy.Time.now() - self.last_attempt_time).to_sec() >= self.timeout:
            rospy.logerr("ALLIGN CHECK || Alignment timeout reached. Stopping rotation and entering standby.")
            self.stop_rotation()
            return

        rospy.loginfo("ALLIGN CHECK || Robot not aligned. Sending rotation command.")
        self.send_rotation_command()

    def send_rotation_command(self):
        twist = Twist()
        twist.angular.z = self.rotation_speed
        self.cmd_vel_pub.publish(twist)
        self.rotation_active = True
        rospy.loginfo("ALLIGN CHECK || Published rotation command with speed: %f", self.rotation_speed)

    def stop_rotation(self):
        if self.rotation_active:
            twist = Twist()  # Stop rotation
            self.cmd_vel_pub.publish(twist)
            self.rotation_active = False
            rospy.loginfo("ALLIGN CHECK || Stopped rotation commands.")

    def reset_alignment_service(self, req):
        rospy.loginfo("ALLIGN CHECK || Resetting alignment attempts. Entering idle mode.")
        self.last_attempt_time = rospy.Time.now()
        self.rotation_active = False
        self.is_active = False
        return TriggerResponse(success=True, message="Alignment reset initiated.")

    def start_alignment_service(self, req):
        rospy.loginfo("ALLIGN CHECK || Starting alignment process.")
        self.is_active = True
        self.last_attempt_time = rospy.Time.now()
        return TriggerResponse(success=True, message="Alignment process started.")

if __name__ == '__main__':
    try:
        RobotAlignmentController()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("ALLIGN CHECK || Robot Alignment Controller node terminated.")