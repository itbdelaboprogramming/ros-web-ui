#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseArray, Pose

def testing_publish_pose_array():
    # Inisialisasi node ROS
    rospy.init_node('pose_array_publisher', anonymous=True)
    
    # Inisialisasi publisher pada topic "/pose_array"
    pub = rospy.Publisher('/pose_array', PoseArray, queue_size=10)
    
    # Set rate untuk publikasi
    rate = rospy.Rate(1)  # 1 Hz
    
    # Buat instance PoseArray
    pose_array = PoseArray()
    pose_array.header.frame_id = "map"  # Set frame id

    # Tambahkan 5 pose ke dalam PoseArray
    for i in range(5):
        pose = Pose()
        pose.position.x = i * 1.0  # Set posisi x berurutan
        pose.position.y = i * 1.5  # Set posisi y berurutan
        pose.position.z = 0.0      # Set posisi z tetap di 0
        
        # Set orientasi menggunakan quaternion identity (tidak ada rotasi)
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 1.0
        
        # Tambahkan pose ke PoseArray
        pose_array.poses.append(pose)

    # Publikasi pose array
    while not rospy.is_shutdown():
        # Update timestamp pada header setiap kali publikasi
        pose_array.header.stamp = rospy.Time.now()
        
        # Publish PoseArray message
        pub.publish(pose_array)
        rospy.loginfo("Published PoseArray with 5 poses.")
        
        # Tunggu hingga rate berikutnya
        rate.sleep()

if __name__ == '__main__':
    try:
        testing_publish_pose_array()
    except rospy.ROSInterruptException:
        pass
