#!/usr/bin/env python3

import rospy
from topic2string.msg import PoseStampedWithInfo 
from geometry_msgs.msg import PoseStamped

def pose_stamped_publisher():
    # Initialize the ROS node
    rospy.init_node('pose_stamped_publisher', anonymous=True)

    # Create a publisher for the /pose_stamped_with_info topic
    pub = rospy.Publisher('/pose_stamped_with_info', PoseStampedWithInfo, queue_size=10)

    # Set a publishing rate
    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        # Create a new PoseStampedWithInfo message
        pose_info_msg = PoseStampedWithInfo()

        # Fill the info field
        pose_info_msg.info.data = "Test info message"  # Example info string

        # Fill the pose field
        pose_info_msg.pose.header.stamp = rospy.Time.now()
        pose_info_msg.pose.header.frame_id = "base_link"  # Adjust frame_id as needed
        pose_info_msg.pose.pose.position.x = 1.0  # Example position
        pose_info_msg.pose.pose.position.y = 2.0
        pose_info_msg.pose.pose.position.z = 0.0
        pose_info_msg.pose.pose.orientation.x = 0.0  # Example orientation
        pose_info_msg.pose.pose.orientation.y = 0.0
        pose_info_msg.pose.pose.orientation.z = 0.0
        pose_info_msg.pose.pose.orientation.w = 1.0

        # Publish the message
        pub.publish(pose_info_msg)
        rospy.loginfo(f"Published: {pose_info_msg}")

        # Sleep for the specified rate
        rate.sleep()

if __name__ == '__main__':
    try:
        pose_stamped_publisher()
    except rospy.ROSInterruptException:
        pass
