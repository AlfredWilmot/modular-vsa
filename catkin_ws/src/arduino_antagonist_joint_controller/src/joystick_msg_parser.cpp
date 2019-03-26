#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/UInt16MultiArray.h"
#include "std_msgs/Bool.h"

#include <iostream>
#include <unistd.h>

/* Fcn declarations */
void test_proximal_joint(const sensor_msgs::JoyConstPtr& msg);
void direct_control();


/* Variables store the parsed gamepad message into global varaibles that controller fcns can access */
static bool A_btn    = 0;

static bool B_btn    = 0;
static bool X_btn    = 0;

static bool LB_btn   = 0;
static bool RB_btn   = 0;

static bool xbox_btn = 0;

static bool at_least_one_change = false;

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
    
    // Left motor packet
    motor_packet.data.push_back(CW);
    motor_packet.data.push_back(0);

    // Right motor packet 
    motor_packet.data.push_back(CW);
    motor_packet.data.push_back(0);

    // Joint toggle
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

/* To filter message repetitions from the joy topic, check that each recieved data of interest is different from it's previous value */
void update_btn_if_changed(bool *btn_to_check, int index, const sensor_msgs::JoyConstPtr& msg)
{
    if(*btn_to_check != msg->buttons[index])
    {
        at_least_one_change = true;
        *btn_to_check = msg->buttons[index];
    }
}

/* Joystick subscriber parses joystick message into relevant variables usable by controller functions */
void test_proximal_joint(const sensor_msgs::JoyConstPtr& msg)
{

    /* Only publish to segment_motor_cmds topic if at least one of the button values have changed, otherwise ignore the message */
    update_btn_if_changed(&A_btn,    0, msg);
    update_btn_if_changed(&B_btn,    1, msg);
    update_btn_if_changed(&X_btn,    2, msg);
    update_btn_if_changed(&LB_btn,   4, msg);
    update_btn_if_changed(&RB_btn,   5, msg);
    update_btn_if_changed(&xbox_btn, 8, msg);



    // A_btn  =   msg->buttons[0];

    // B_btn  =   msg->buttons[1];
    // X_btn  =   msg->buttons[2];

    // LB_btn =   msg->buttons[4];
    // RB_btn =   msg->buttons[5];

    // xbox_btn = msg->buttons[8];         

    // Testing direct control
    

    if (at_least_one_change) 
    {
        // reset filter flag
        at_least_one_change = false;

        // filter button-bounce
        usleep(1000);
        
        // publish control messages according to control function below 
        direct_control();
    }
    else
    {
        return;
    }
    
}

/* A controller that uses X,A,B,LB, & RB buttons for direct control of proximal joint of a single connected segment */
void direct_control()
{
    //Right motor rotates:  CW if A is pressed or if B is pressed:                 A || B
    //                      CCW if A && RB are pressed or if B && RB are pressed:  RB && (A || B)

    //Left motor roates:    CW if A is pressed or if X is pressed:                 A || X
    //                      CCW if A && LB are pressed or if X && LB are pressed:  LB && (A || X) 



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

    /* Toggle which joint is driven by the motor packet */
    if(xbox_btn)
    {

        motor_packet.data.at(4) = !motor_packet.data.at(4);

    }

    //publish updated packet 
    motor_pub.publish(motor_packet);
}