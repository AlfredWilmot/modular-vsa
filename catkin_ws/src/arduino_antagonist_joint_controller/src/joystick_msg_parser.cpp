#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/UInt16MultiArray.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/Bool.h"

#include <iostream>
#include <unistd.h>

/* Fcn declarations */
void test_proximal_joint(const sensor_msgs::JoyConstPtr& msg);
void parse_for_joystick_and_triggers(const sensor_msgs::JoyConstPtr& msg);

void direct_control();
void joy_stick_OL_control();

uint8_t map_float_to_UInt8(float, float, float);

/* Store parsed button presses */
static bool A_btn    = 0;
static bool B_btn    = 0;
static bool X_btn    = 0;
static bool LB_btn   = 0;
static bool RB_btn   = 0;
static bool xbox_btn = 0;

/* Store parsed axis position values */
static uint8_t right_joy_roll = 0;      //axes index: 4
static uint8_t right_joy_pitch = 0;     //axes index: 3
static uint8_t right_trigger  = 0;      //axes index: 5
static uint8_t left_trigger   = 0;      //axes index: 2


/* Flag ensures that parsed identical gamepad messages are only published once */
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
    int packet_len = 12;

    for(int i = 0; i < packet_len; i++)
    {
        motor_packet.data.push_back(0);
    }

    /* ROS Node Premable */
    ros::init(argc, argv, "joystick_msg_parser");
    ros::NodeHandle nh;

    /* Subscribe to gamepad /joy topic (callback filters joy-msg) */
    ros::Subscriber sub = nh.subscribe("joy", 1, parse_for_joystick_and_triggers);
   
   
    /* Publish filtered joy-msg into something simple array for the MCU */
    motor_pub = nh.advertise<std_msgs::UInt16MultiArray>("segment_motor_cmds", 1);

    /* Service any subscriber callbacks (publishing parsed data takes place within callback) */
    ros::spin();

    return 0;
}

/************************/
/* Filtering functions */
/**********************/

/* To filter message repetitions from the joy topic, check that each recieved data of interest is different from it's previous value */
void update_btn_if_changed(bool *btn_to_check, int index, const sensor_msgs::JoyConstPtr& msg)
{
    if(*btn_to_check != msg->buttons[index])
    {
        at_least_one_change = true;
        *btn_to_check = msg->buttons[index];
    }
}

/* Only indicate change of gamepad msg, once beyond a sufficient hysteresis band around previous reading */
void update_axis_if_changed(float *axis_to_check, float hyst, int index, const sensor_msgs::JoyConstPtr& msg)
{
    float diff = abs(*axis_to_check - msg->axes[index]);

    if(diff > hyst)
    {
        at_least_one_change = true;
        *axis_to_check = msg->axes[index];
    }
}

/*****************************/
/* joy-msg parser functions */
/***************************/

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

/* Joystick subscriber parses joystick msg to direct digital pin & PWM pin allocation on MCU*/
// Generates control packet of form: "[IN1, IN2, ENA, IN3, IN4, ENB]" for each joint.
void parse_for_joystick_and_triggers(const sensor_msgs::JoyConstPtr& msg)
{
    //update_axis_if_changed();
}

/*************************/
/* Controller functions */
/***********************/

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
        motor_packet.data.at(1) = 255;  //255/2 = ~50% duty
        
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
        motor_packet.data.at(3) = 255;
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

/* A controller that uses Right-joyStick, as well as LT & RT for tension control of either joint */
void joy_stick_OL_control()
{

}


/*******************/
/* Misc Functions */
/*****************/

/* Simply scales any input float to a correspinding UInt8 value (to drive MCU PWM) for motor duty value. Assumes input range of: [-1, 1]*/
uint8_t map_float_to_UInt8(float input_val, float min_input = -1, float max_input = 1)
{
    return (uint8_t) round((input_val - min_input) * 255 / (max_input - min_input) );
}