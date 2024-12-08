#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Bool
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


        rospy.loginfo("Loaded particle_align_params: %s", params)

        self.std_dev_threshold = params.get('std_dev_threshold', 0.5)
        self.allign_topic = params.get('allign_topic', '/client/allign')
        self.particlecloud_topic = params.get('particlecloud_topic', '/particlecloud')

        # ROS publishers and subscribers
        self.allign_pub = rospy.Publisher(self.allign_topic, Bool, queue_size=10)
        rospy.Subscriber(self.particlecloud_topic, PoseArray, self.particlecloud_callback)

        rospy.loginfo("Node initialized with std_dev_threshold: %f", self.std_dev_threshold)

    def particlecloud_callback(self, msg):
        # Extract positions from PoseArray
        positions = np.array([[pose.position.x, pose.position.y] for pose in msg.poses])
        
        if len(positions) < 2:
            rospy.logwarn("Insufficient data points in particlecloud. Skipping.")
            return

        # Calculate standard deviation for x and y
        std_dev = np.std(positions, axis=0)
        max_std_dev = np.max(std_dev)

        rospy.loginfo("Calculated std_dev: %f", max_std_dev)

        # Check alignment
        is_alligned = max_std_dev < self.std_dev_threshold
        self.allign_pub.publish(is_alligned)

        rospy.loginfo("Alignment status published: %s", is_alligned)

if __name__ == '__main__':
    try:
        validator = ParticleAlignValidator()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Particle Align Validator node terminated.")
