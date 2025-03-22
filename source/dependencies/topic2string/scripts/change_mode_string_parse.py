#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from msd700_webui_msg.msg import SwitchModeMsg

class SwitchModeStringNode(object):
    def __init__(self):
        # Inisialisasi node ROS
        rospy.init_node('switch_mode_node', anonymous=False)
    
        # ======================= PARAMETER SERVER =======================
        # Change the default values of the following parameters in the launch file.
        # The default values are used if the parameters are not found in the parameter server.
        # The parameter server can be set using the launch file or the command line.
        # Make sure to sync the parameter names with the web interface.
        switchmode_topic = rospy.get_param('~switchmode_topic', '/server/switch_mode')
        command_topic = rospy.get_param('~command_topic', '/command/switch_mode')
        # ======================= PARAMETER SERVER =======================
        
        # Detailed log info
        rospy.loginfo("SwitchModeStringNode is active")
        rospy.loginfo("     switchmode_topic to: %s", switchmode_topic)
        rospy.loginfo("     command_topic to: %s", command_topic)

        # Inisialisasi publisher untuk topik switchmode_topic
        self.publisher = rospy.Publisher(switchmode_topic, SwitchModeMsg, queue_size=10)
        
        # Inisialisasi subscriber untuk topik command_topic
        self.subscriber = rospy.Subscriber(command_topic, String, self.callback)
        
        rospy.loginfo("SwitchModeStringNode activated.")

    def callback(self, msg):
        """
        Callback to process the string message received from /command/switch_mode.
        The message is split using the '#' delimiter and converted into a SwitchModeMsg message.
        """
        try:
            # Memecah pesan berdasarkan delimiter '#'
            parts = msg.data.split('#')
            if len(parts) != 5:
                rospy.logerr("Message format is incorrect. Expected 5 parts, received: %d", len(parts))
                return

            # Parsing setiap bagian pesan
            mode = parts[0]
            open_rviz = self.str_to_bool(parts[1])
            use_simulator = self.str_to_bool(parts[2])
            map_file = parts[3]
            point_mode = parts[4]

            # Membuat instance pesan SwitchModeMsg
            switch_mode_msg = SwitchModeMsg()
            switch_mode_msg.mode = mode
            switch_mode_msg.open_rviz = open_rviz
            switch_mode_msg.use_simulator = use_simulator
            switch_mode_msg.map_file = map_file
            switch_mode_msg.point_mode = point_mode

            # Mempublish pesan ke topik /switchmode
            self.publisher.publish(switch_mode_msg)
            rospy.loginfo("SwitchModeMsg message has been published: %s", switch_mode_msg)
        except Exception as e:
            rospy.logerr("An error occurred while processing the message: %s", str(e))

    def str_to_bool(self, s):
        """
        Converts a string to a boolean value.
        Treats 'true', '1', or 'yes' (case insensitive) as True.
        """
        return s.strip().lower() in ['true', '1', 'yes']

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = SwitchModeStringNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
