#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import Bool

class RobotAlignmentController:
    def __init__(self):
        rospy.init_node('robot_alignment_controller', anonymous=False)

        # Load parameters
        self.check_frequency = rospy.get_param('~check_frequency', 1.0)  # Frequency in Hz
        self.timeout = rospy.get_param('~timeout', 30.0)  # Timeout in seconds
        self.rotation_speed = rospy.get_param('~rotation_speed', 0.5)  # Rotation speed for z-axis
        self.alignment_topic = rospy.get_param('~alignment_topic', '/check_alignment')
        self.cmd_vel_topic = rospy.get_param('~cmd_vel_topic', '/mux/allign')

        # State variables
        self.aligned = False
        self.rotation_active = False
        self.last_attempt_time = rospy.Time.now()

        # Publishers and Service
        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=10)
        rospy.Service('/reset_alignment', Trigger, self.reset_alignment_service)

        # Subscriber to alignment status
        rospy.Subscriber('/client/allign', Bool, self.alignment_status_callback)

        # Timer for checking alignment
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.check_frequency), self.check_alignment)

        rospy.loginfo("Robot Alignment Controller initialized with:")
        rospy.loginfo("  Check frequency: %f Hz", self.check_frequency)
        rospy.loginfo("  Timeout: %f seconds", self.timeout)

    def alignment_status_callback(self, msg):
        self.aligned = msg.data
        rospy.loginfo("Alignment status updated: %s", self.aligned)

    def check_alignment(self, event):
        if self.aligned:
            rospy.loginfo("Robot already aligned. No action required.")
            self.stop_rotation()
            return

        if (rospy.Time.now() - self.last_attempt_time).to_sec() >= self.timeout:
            rospy.logerr("Alignment timeout reached. Stopping rotation and entering standby.")
            self.stop_rotation()
            return

        rospy.loginfo("Robot not aligned. Sending rotation command.")
        self.send_rotation_command()

    def send_rotation_command(self):
        twist = Twist()
        twist.angular.z = self.rotation_speed
        self.cmd_vel_pub.publish(twist)
        self.rotation_active = True
        rospy.loginfo("Published rotation command with speed: %f", self.rotation_speed)

    def stop_rotation(self):
        if self.rotation_active:
            twist = Twist()  # Stop rotation
            self.cmd_vel_pub.publish(twist)
            self.rotation_active = False
            rospy.loginfo("Stopped rotation commands.")

    def reset_alignment_service(self, req):
        rospy.loginfo("Resetting alignment attempts.")
        self.last_attempt_time = rospy.Time.now()
        self.rotation_active = False
        return TriggerResponse(success=True, message="Alignment reset initiated.")

if __name__ == '__main__':
    try:
        RobotAlignmentController()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Robot Alignment Controller node terminated.")
