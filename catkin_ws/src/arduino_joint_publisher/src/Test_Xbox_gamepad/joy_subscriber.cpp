/* cpp subscriber tutorial found here: https://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29#roscpp_tutorials.2BAC8-Tutorials.2BAC8-WritingPublisherSubscriber.Writing_the_Subscriber_Node*/

/* ROS libs */
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Float64.h"

/* STD libs */
#include <iostream>

/* Function declarations */
void sniffer_callback(const sensor_msgs::JoyConstPtr& msg);
void test_proximal_joint(const sensor_msgs::JoyConstPtr& msg);

/* Subscribe to the xbox-360 joy-stick topic, and print info to terminal */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "joy_stick_sniffer");

    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("joy", 0, test_proximal_joint);

    ros::spin();
    
    return 0;
}

/* TEST: simply print right-joystick values to terminal */
void sniffer_callback(const sensor_msgs::JoyConstPtr& msg)
{
    std::cout << "Right-stick Roll:  " << msg->axes[3] << "\n"; //Right: < 0, Left > 0
    std::cout << "Right-stick Pitch: " << msg->axes[4] << "\n"; //Up:    > 0, Down < 0  
       
}

/* Joystick subscriber callback for controlling motors of one joint */
void test_proximal_joint(const sensor_msgs::JoyConstPtr& msg)
{
    bool A_btn  = msg->buttons[0];

    bool B_btn  = msg->buttons[1];
    bool X_btn  = msg->buttons[2];

    bool LB_btn = msg->buttons[4];
    bool RB_btn = msg->buttons[5];


    //Right motor rotates:  CW if A is pressed or if B is pressed:                 A || B
    //                      CCW if A && RB are pressed or if B && RB are pressed:  RB && (A || B)

    //Left motor roates:    CW if A is pressed or if X is pressed:                 A || X
    //                      CCW if A && LB are pressed or if X && LB are pressed:  LB && (A || X) 
    
    std::cout << "Right motor";

    if(A_btn || B_btn)
    {
        RB_btn ? std::cout << " CCW\n" : std::cout << " CW\n";
    }
    else
    {
        std::cout << " stop\n";
    }



    std::cout << "Left motor";

    if(A_btn || X_btn)
    {
        LB_btn ? std::cout << " CCW\n" : std::cout << " CW\n";
    }
    else
    {
        std::cout << " stop\n";
    }
    
    
}


/* Relationship between joystick inputs and joystick message */

// [A, B, X, Y, LB, RB, BCK, STRT, XBX, Lft-Stk, Rgt-Stk] : Binary-btns on gamepad
// [0, 1, 2, 3, 4,  5,  6,   7,    8,   9,       10     ] : Binary-btns array index

// [Lft-Stk (Rgt->Lft), Lft-Stk (Dwn->Up), Lft-Trgr (Clsd->Opnd), ...]
// [0        (-1->1)  , 1       (-1->1)  , 2        (-1->1)     , ...]


// Example: 'B' drives Right-motor CW (holding RB && 'B' makes it go CCW)
//          'X' drives Left-motor  CW (holding LB && 'X' makes it go CCW)