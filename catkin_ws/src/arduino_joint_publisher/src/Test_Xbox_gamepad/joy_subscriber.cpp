/* cpp subscriber tutorial found here: https://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29#roscpp_tutorials.2BAC8-Tutorials.2BAC8-WritingPublisherSubscriber.Writing_the_Subscriber_Node*/

/* ROS libs */
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Float64.h"

/* STD libs */
#include <iostream>

/* Function declarations */
void sniffer_callback(const sensor_msgs::JoyConstPtr& msg);


/* Subscribe to the xbox-360 joy-stick topic, and print info to terminal */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "joy_stick_sniffer");

    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("joy", 1, sniffer_callback);

    ros::spin();
    
    return 0;
}

/* TEST: simply print right-joystick values to terminal */
void sniffer_callback(const sensor_msgs::JoyConstPtr& msg)
{
    std::cout << "Right-stick Roll:  " << msg->axes[3] << "\n"; //Right: < 0, Left > 0
    std::cout << "Right-stick Pitch: " << msg->axes[4] << "\n"; //Up:    > 0, Down < 0     
}