#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>

class EmergencyStopNode {
private:
    ros::NodeHandle nh_{"~"};
    ros::Subscriber pause_sub_;
    ros::Publisher cmd_pub_;

    bool pause_flag_ = false;
    double publish_frequency_;

    bool is_status_changed_ = true;

    geometry_msgs::Twist zero_twist_;

public:
    EmergencyStopNode() : nh_{"~"} { 
        std::string pause_topic, emergency_topic;
        
        // Load parameters with proper namespace
        nh_.param("pause_topic", pause_topic, std::string("/mapping/Pause"));
        nh_.param("emergency_topic", emergency_topic, std::string("/cmd_emergency"));
        nh_.param("publish_frequency", publish_frequency_, 10.0);

        // Menginisialisasi subscribers dan publisher
        pause_sub_ = nh_.subscribe(pause_topic, 1, &EmergencyStopNode::pauseCallback, this);
        cmd_pub_ = nh_.advertise<geometry_msgs::Twist>(emergency_topic, 1);

        // Menginisialisasi zero_twist_
        zero_twist_.linear.x = 0.0;
        zero_twist_.linear.y = 0.0;
        zero_twist_.linear.z = 0.0;
        zero_twist_.angular.x = 0.0;
        zero_twist_.angular.y = 0.0;
        zero_twist_.angular.z = 0.0;

        // Print Roscpp info node initialized
        ROS_INFO("Emergency Stop Node Initialized with :");
        ROS_INFO("      Pause Topic : %s", pause_topic.c_str());
        ROS_INFO("      Emergency Topic : %s", emergency_topic.c_str());
        ROS_INFO("      Publish Frequency : %.2f", publish_frequency_);
    }

    void pauseCallback(const std_msgs::Bool::ConstPtr& msg) {
        pause_flag_ = msg->data;

        is_status_changed_ = true;
    }

    void run() {
        ros::Rate rate(publish_frequency_);
        while (ros::ok()) {
            if(pause_flag_) {
                cmd_pub_.publish(zero_twist_); // Publish zero Twist
                if(is_status_changed_) {
                    ROS_INFO("Pause signal active. Publishing zero Twist.");
                    is_status_changed_ = false;
                }
            } else {
                if(is_status_changed_) {
                    ROS_INFO("Pause signal inactive.");
                    is_status_changed_ = false;
                }
            }

            ros::spinOnce();
            rate.sleep();
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "emergency_stop_node");
    EmergencyStopNode emergency_stop_node;
    emergency_stop_node.run();
    return 0;
}
