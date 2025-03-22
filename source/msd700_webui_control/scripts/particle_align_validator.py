#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Bool
from std_srvs.srv import Trigger, TriggerResponse
import numpy as np

class ParticleAlignValidator:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('particle_align_validator', anonymous=False)

        # Get nested parameters
        params = rospy.get_param('/particle_align_params', None)
        if params is None:
            rospy.logerr("Parameter '/particle_align_params' not loaded. Check your YAML file and launch file.")
            rospy.signal_shutdown("Missing parameters.")
            return

        self.std_dev_threshold = params.get('std_dev_threshold', 0.5)
        self.allign_topic = params.get('allign_topic', '/client/allign')
        self.particlecloud_topic = params.get('particlecloud_topic', '/particlecloud')
        self.check_allign_service = params.get('check_allign_service', '/check_alignment')

        # ROS publishers and subscribers
        self.allign_pub = rospy.Publisher(self.allign_topic, Bool, queue_size=10)
        rospy.Subscriber(self.particlecloud_topic, PoseArray, self.particlecloud_callback)

        # Initialize service
        self.is_alligned = False  # Default alignment status
        rospy.Service(self.check_allign_service, Trigger, self.handle_check_alignment)
        
        # Print Info all Parameters
        rospy.loginfo("Particle Align Validator initialized with:")
        rospy.loginfo("     std_dev_threshold: %f", self.std_dev_threshold)
        rospy.loginfo("     Allign topic: %s", self.allign_topic)
        rospy.loginfo("     Particlecloud topic: %s", self.particlecloud_topic)
        rospy.loginfo("     Check allign service: %s", self.check_allign_service)

    def particlecloud_callback(self, msg):
        # Extract positions from PoseArray
        positions = np.array([[pose.position.x, pose.position.y] for pose in msg.poses])

        if len(positions) < 2:
            rospy.logwarn("PARTICLE ALLIGN || Insufficient data points in particlecloud. Skipping.")
            return

        # Calculate standard deviation for x and y
        std_dev = np.std(positions, axis=0)
        max_std_dev = np.max(std_dev)

        rospy.loginfo("PARTICLE ALLIGN || Calculated std_dev: %f", max_std_dev)

        # Check alignment
        self.is_alligned = max_std_dev < self.std_dev_threshold
        self.allign_pub.publish(self.is_alligned)

        rospy.loginfo("PARTICLE ALLIGN || Alignment status published: %s", self.is_alligned)

    def handle_check_alignment(self, req):
        # Handle service request to check alignment status
        return TriggerResponse(
            success=self.is_alligned,
            message=f"PARTICLE ALLIGN || Alignment status: {'Aligned' if self.is_alligned else 'Not Aligned'}"
        )

if __name__ == '__main__':
    try:
        validator = ParticleAlignValidator()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("PARTICLE ALLIGN || Particle Align Validator node terminated.")
