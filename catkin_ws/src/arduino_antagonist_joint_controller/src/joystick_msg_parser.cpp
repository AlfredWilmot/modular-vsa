#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/UInt16MultiArray.h"
#include "std_msgs/Bool.h"

#include <iostream>


/* Fcn declarations */
void test_proximal_joint(const sensor_msgs::JoyConstPtr& msg);
void direct_control();


/* Variables store the parsed gamepad message into global varaibles that controller fcns can access */
static bool A_btn  = 0;

static bool B_btn  = 0;
static bool X_btn  = 0;

static bool LB_btn = 0;
static bool RB_btn = 0;


#define CW  0
#define CCW 1

/* motor msg */
static std_msgs::UInt16MultiArray motor_packet;

/* motor msg publisher */
static ros::Publisher motor_pub;


/* Subscribe to the xbox-360 joy-stick topic, 
   publish corresponding motor control msg to "/segment_motor_cmds" topic */
int main(int argc, char **argv)
{
    /* Initializing motor packet*/

    motor_packet.data.clear();
    // direction
    motor_packet.data.push_back(CW);
    motor_packet.data.push_back(CW);
    //duty
    motor_packet.data.push_back(0);
    motor_packet.data.push_back(0);


    /* ROS Node Premable */
    ros::init(argc, argv, "joystick_msg_parser");

    ros::NodeHandle nh;
    /* Subscribe to gamepad /joy topic (callback filters joy-msg) */
    ros::Subscriber sub = nh.subscribe("joy", 1, test_proximal_joint);
    /* Publish filtered joy-msg into something simple array for the MCU */
    motor_pub = nh.advertise<std_msgs::UInt16MultiArray>("segment_motor_cmds", 1);



    /* Service any subscriber callbacks */
    ros::spin();

  return 0;
}




/* Joystick subscriber parses joystick message into relevant variables usable by controller functions */
void test_proximal_joint(const sensor_msgs::JoyConstPtr& msg)
{
    A_btn  = msg->buttons[0];

    B_btn  = msg->buttons[1];
    X_btn  = msg->buttons[2];

    LB_btn = msg->buttons[4];
    RB_btn = msg->buttons[5];

    // Testing direct control
    direct_control();
        
}

/* A controller that uses X,A,B,LB, & RB buttons for direct control of proximal joint of a single connected segment */
void direct_control()
{
    //Right motor rotates:  CW if A is pressed or if B is pressed:                 A || B
    //                      CCW if A && RB are pressed or if B && RB are pressed:  RB && (A || B)

    //Left motor roates:    CW if A is pressed or if X is pressed:                 A || X
    //                      CCW if A && LB are pressed or if X && LB are pressed:  LB && (A || X) 




    //std::cout << "Right motor";
    

    if(A_btn || B_btn)
    {
        // set dir
        //RB_btn ? std::cout << " CCW\n" : std::cout << " CW\n";
        RB_btn ? motor_packet.data.at(0) = CCW : motor_packet.data.at(0) = CW;

        // set duty
        motor_packet.data.at(1) = 128;  //255/2 = ~50% duty
        
    }
    else
    {
        //std::cout << " stop\n";
        motor_packet.data.at(1) = 0;
    }



    std::cout << "Left motor";

    if(A_btn || X_btn)
    {
        //set dir
        //LB_btn ? std::cout << " CCW\n" : std::cout << " CW\n";
        LB_btn ? motor_packet.data.at(2) = CCW : motor_packet.data.at(2) = CW;

        // set duty 
        motor_packet.data.at(3) = 128;
    }
    else
    {
        //std::cout << " stop\n";
        motor_packet.data.at(3) = 0;
    }

    //publish updated packet 
    motor_pub.publish(motor_packet);
}