#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/UInt16MultiArray.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/Bool.h"
#include "string.h"

#include <iostream>
#include <unistd.h>

/*------------------------*/
/* Function declarations */
/*----------------------*/

//Subscriber callback for Open-Loop control using gamepad
void OL_control(const sensor_msgs::JoyConstPtr& msg);

//Closed-loop control for maintaining current position
void CL_control();
void reset_all_motors();

// Functions to directly control joint using D-pad & co-contract with left-trigger.
int proximal_joint_cmds(const sensor_msgs::JoyConstPtr msg);
int distal_joint_cmds(const sensor_msgs::JoyConstPtr msg);

// Prevent duplicate control packets from being sent to MCU, and also prevents expired packages from persisting.
int filter_duplicate_packets();

// Control motors directly (send duty, and direction: loosen/ tighten)
int motor_override(int duty, std::string motor_name, int contract_tendon);


// cycles through the array of /joy msg indicies to check if btns or axes are in use.
int check_btns(int *gamepad_btns_to_check, const sensor_msgs::JoyConstPtr msg)
{

    int array_length = sizeof(gamepad_btns_to_check)/sizeof(*gamepad_btns_to_check);
    
    ROS_INFO("Checking for %d buttons.\n", array_length);

    for(size_t i = 0; i < array_length; i++)
    {
        if(msg->buttons.at(i) != 0)
        {
            return 1;
        }
    }

    return 0;
}

/*------------*/
/* motor msg */
/*----------*/

static std_msgs::UInt16MultiArray motor_packet;
static std_msgs::UInt16MultiArray previous_motor_packet;

static int packet_length = 12; // packet is 8 elements long (two per motor in single segment) + 4 motor-duty values.

/* motor msg publisher */
static ros::Publisher motor_pub;


/*-----------------------------------------*/
/* defining gamepad button/ axis indicies */
/*---------------------------------------*/

// AXIS (-1.0, 1.0):
const int d_pad_up_down         = 7; // U = +1.0, D = -1.0
const int d_pad_left_right      = 6; // L = +1.0, R = -1.0
const int right_trigger         = 5; // UnPressed = -1.0, Pressed = +1.0 (initial value, before ever being pressed, will show as 0.0)
const int right_joy_up_down     = 4; // U = +1.0, D = -1.0
const int right_joy_left_right  = 3; // L = +1.0, R = -1.0
const int left_trigger          = 2; // UnPressed = -1.0, Pressed = +1.0 (initial value, before ever being pressed, will show as 0.0)   
const int left_joy_up_down      = 1; // U = +1.0, D = -1.0
const int left_joy_left_right   = 0; // L = +1.0, R = -1.0

// BUTTONS (default 0, pressed 1):
const int press_right_joy   = 10;
const int press_left_joy    = 9;
const int xbox_btn          = 8;
const int start_btn         = 7;
const int back_btn          = 6;
const int RB_btn            = 5;
const int LB_btn            = 4;
const int Y_btn             = 3;
const int X_btn             = 2;
const int B_btn             = 1;
const int A_btn             = 0;



/*-------------------------------------------------------------------------------*/
/* global variables for the joint controller, and relevant subscriber callbacks */
/*-----------------------------------------------------------------------------*/

// Encoder values.
static int initial_proximal_joint_value = 0;

static int initial_distal_joint_value = 0;

static int proximal_joint_value = 0;
static int distal_joint_value = 0;

// Encoder subscriber callbacks 
void grab_proximal_encoder_value(const std_msgs::UInt16ConstPtr msg);
void grab_distal_encoder_value(const std_msgs::UInt16ConstPtr msg);

// I-sense values.
static int proximal_motorA_load;
static int proximal_motorB_load;
static int distal_motorA_load;
static int distal_motorB_load;

// I-sense subscriber callbacks
void grab_proximal_motorA_load(const std_msgs::UInt16ConstPtr msg);
void grab_proximal_motorB_load(const std_msgs::UInt16ConstPtr msg);
void grab_distal_motorA_load(const std_msgs::UInt16ConstPtr msg);
void grab_distal_motorB_load(const std_msgs::UInt16ConstPtr msg);


// Various experimental states controlled by gamepad toggle-switches.
bool CL_control_mode_is_active = false;
bool D_pad_in_use = false;


/*------------*/
/* Main Loop */
/*----------*/

/* Subscribe to the xbox-360 joy-stick topic,publish corresponding motor control msg to "/segment_motor_cmds" topic */
int main(int argc, char **argv)
{
    /* Initializing motor packet */ 
    motor_packet.data.clear();
    previous_motor_packet.data.clear();



    for(int i = 0; i < packet_length; i++)
    {
        motor_packet.data.push_back(0);     // Initialized packet with zero-value.
        previous_motor_packet.data.push_back(0);
    }

    /* ROS Node Premable */
    ros::init(argc, argv, "joystick_msg_parser");
    ros::NodeHandle nh;

    /* Subscribe to gamepad /joy topic */
    ros::Subscriber sub = nh.subscribe("joy", 1, OL_control);
    
    ros::Subscriber proximal_encoder_sub    = nh.subscribe("proximal_encoder", 1, grab_proximal_encoder_value);
    ros::Subscriber distal_encoder_sub      = nh.subscribe("distal_encoder",   1, grab_distal_encoder_value);

    ros::Subscriber proximal_motorA_load    = nh.subscribe("I_sense_1", 1, grab_proximal_motorA_load);
    ros::Subscriber proximal_motorB_load    = nh.subscribe("I_sense_3", 1, grab_proximal_motorB_load);
    ros::Subscriber distal_motorA_load      = nh.subscribe("I_sense_2", 1, grab_distal_motorA_load);
    ros::Subscriber distal_motorB_load      = nh.subscribe("I_sense_4", 1, grab_distal_motorB_load);
   
    /* Publish filtered joy-msg into binary values that will be allocated by MCU as digital pin output states (thereby controlling motor direction) */
    motor_pub = nh.advertise<std_msgs::UInt16MultiArray>("segment_motor_cmds", 1);


    ros::Rate loop_rate(50); //10Hz loop rate

    while(ros::ok())
    {
        
        if(CL_control_mode_is_active && !D_pad_in_use) // if gamepad is not in use, and the CL control mode has been activated, run CL control to hold current position.
        {
            CL_control();
        }

        /* Service any subscriber callbacks (publishing parsed data takes place within callback) */
        ros::spinOnce();

        /* Enforces loop period to the designated rate (only works properly if all processes preceeding sleep() have a lower latency than 1/loop_rate). */
        loop_rate.sleep();
    }
    

    return 0; // exit when roscore dies
}



/* Define Proximal joint movement */
int proximal_joint_cmds(const sensor_msgs::JoyConstPtr msg)
{

    if(msg->axes[d_pad_left_right] == 1.0)
    {
        //CW
        motor_override(204, "distal_motor_A", 1);
        motor_override(204, "distal_motor_B", 0);

        //Co-contract if trigger is pressed 
        if(msg->axes[left_trigger] < 0.0)
        {
            motor_packet.data.at(2) = !motor_packet.data.at(0);
            motor_packet.data.at(3) = !motor_packet.data.at(1);   
        }

        return 1; // indicate gamepad was used by operator.

    }
    else if(msg->axes[d_pad_left_right] == -1.0)
    {
        //CCW
        motor_override(204, "distal_motor_A", 0);
        motor_override(204, "distal_motor_B", 1);

        //Co-contract if trigger is pressed 
        if(msg->axes[left_trigger] < 0.0)
        {
            motor_packet.data.at(0) = !motor_packet.data.at(2);
            motor_packet.data.at(1) = !motor_packet.data.at(3);   
        }

        return 1; // indicate gamepad was used by operator.

    }
    else
    {
        //Brake motor if D-pad is not pressed 
        motor_packet.data.at(0) = 0;
        motor_packet.data.at(1) = 0;
        motor_packet.data.at(2) = 0;
        motor_packet.data.at(3) = 0;         
    }

    return 0; // indicate gamepad was not used by operator.

}

/* Define Distal joint movement */
int distal_joint_cmds(const sensor_msgs::JoyConstPtr msg)
{

    if(msg->axes[d_pad_up_down] == 1.0)
    {
        //CW
        motor_override(204, "proximal_motor_A", 1);
        motor_override(204, "proximal_motor_B", 0);

        //Co-contract if trigger is pressed 
        if(msg->axes[left_trigger] < 0.0)
        {
            motor_packet.data.at(4) = !motor_packet.data.at(6);
            motor_packet.data.at(5) = !motor_packet.data.at(7);   
        }

        return 1; // indicate gamepad was used by operator.

    }
    else if(msg->axes[d_pad_up_down] == -1.0)
    {
        //CCW
        motor_override(204, "proximal_motor_A", 0);
        motor_override(204, "proximal_motor_B", 1);

        //Co-contract if trigger is pressed 
        if(msg->axes[left_trigger] < 0.0)
        {
            motor_packet.data.at(6) = !motor_packet.data.at(4);
            motor_packet.data.at(7) = !motor_packet.data.at(5);   
        }

        return 1; // indicate gamepad was used by operator.

    }
    else
    {
        //Brake motor if D-pad is not pressed 

        motor_packet.data.at(4) = 0;
        motor_packet.data.at(5) = 0;
        motor_packet.data.at(6) = 0;
        motor_packet.data.at(7) = 0;

        return 0; // indicate gamepad was not used by operator.
    }
}

/* Direct control of individual motor */
int motor_override(int duty, std::string motor_name, int contract_tendon)
{

    if(std::strcmp(motor_name.c_str(), "proximal_motor_A") == 0)
    {
        if(contract_tendon)
        {
            // CW = tighten
            motor_packet.data.at(4) = 1;
            motor_packet.data.at(5) = 0;
        }
        else
        {
             // CCW = loosen
            motor_packet.data.at(4) = 0;
            motor_packet.data.at(5) = 1;           
        }

        // set motor speed
        motor_packet.data.at(8) = duty;

    }
    else if (std::strcmp(motor_name.c_str(), "proximal_motor_B") == 0) 
    {
        if(contract_tendon)
        {
            // CCW = tighten
            motor_packet.data.at(6) = 0;
            motor_packet.data.at(7) = 1;
        }
        else
        {
             // CW = loosen
            motor_packet.data.at(6) = 1;
            motor_packet.data.at(7) = 0;          
        }

        // set motor speed
        motor_packet.data.at(10) = duty;

    }
    else if (std::strcmp(motor_name.c_str(), "distal_motor_A") == 0) 
    {
        if(contract_tendon)
        {
            // CCW = tighten
            motor_packet.data.at(0) = 1;
            motor_packet.data.at(1) = 0;
        }
        else
        {
             // CW = loosen
            motor_packet.data.at(0) = 0;
            motor_packet.data.at(1) = 1;          
        }

        // set motor speed
        motor_packet.data.at(9) = duty;

    }
    else if (std::strcmp(motor_name.c_str(), "distal_motor_B") == 0) 
    {
        if(contract_tendon)
        {
            // CCW = tighten
            motor_packet.data.at(2) = 0;
            motor_packet.data.at(3) = 1;
        }
        else
        {
             // CW = loosen
            motor_packet.data.at(2) = 1;
            motor_packet.data.at(3) = 0;          
        }

        // set motor speed
        motor_packet.data.at(11) = duty;
    }
    else 
    {   
        ROS_INFO("motor ID for overried not recognized.\n");
        return -1; //indicate badness
    }

    ROS_INFO("%s: Co-contracting (%d), with %3.1f%% duty\n", motor_name.c_str(), contract_tendon, ((float(duty))/255)*100);
    
    return 0; //indicate goodness
     

}

/* Filtering out duplicate joy messages */
int filter_duplicate_packets()
{

    bool is_not_duplicate_packet = false;

    for(int i=0; i<packet_length; i++)
    {   
        // if any of the new packet elements are different to the previous corresponding packet element, then update the old packet and set flag.
        if(previous_motor_packet.data.at(i) != motor_packet.data.at(i))
        {
            is_not_duplicate_packet = true;
            previous_motor_packet.data.at(i) = motor_packet.data.at(i);
        }
    }

    // Only Publish the assemlbed motor-control data-packet if it is different from the previous one.
    if(is_not_duplicate_packet)
    {
        return 0; // indicates new packet is not a duplicate.
    }
    else
    {
        return -1;  // indicates packet is a duplicate!
    }
       
}


/* Put all motors into brake mode */
void reset_all_motors()
{
    for(int i=0; i<packet_length; i++)
    {   
       motor_packet.data.at(i) = 0;
    }

    motor_pub.publish(motor_packet);
}

/*--------------------------------*/
/* Subscriber callback functions */
/*------------------------------*/

// Encoder subscriber callbacks (grab data frames)
void grab_proximal_encoder_value(const std_msgs::UInt16ConstPtr msg)
{
    if (initial_proximal_joint_value == 0) 
    {
        initial_proximal_joint_value = msg->data;
    }

    proximal_joint_value = msg->data;

}

void grab_distal_encoder_value(const std_msgs::UInt16ConstPtr msg)
{
    if (initial_distal_joint_value == 0) 
    {
        initial_distal_joint_value = msg->data;
    }

    distal_joint_value = msg->data;
}


// I-sense subscriber callbacks (grab data frames)
void grab_proximal_motorA_load(const std_msgs::UInt16ConstPtr msg)
{
    proximal_motorA_load = msg->data;
}

void grab_proximal_motorB_load(const std_msgs::UInt16ConstPtr msg)
{
    proximal_motorB_load = msg->data;
}
void grab_distal_motorA_load(const std_msgs::UInt16ConstPtr msg)
{
    distal_motorA_load = msg->data;
}
void grab_distal_motorB_load(const std_msgs::UInt16ConstPtr msg)
{
    distal_motorB_load = msg->data;
}



/* A controller that uses Right-joyStick, as well as LT & RT for tension control of either joint */
void OL_control(const sensor_msgs::JoyConstPtr& msg)
{   

    // Toggle CL control mode by pressing xbox btn.
    if (msg->buttons[xbox_btn]) 
    {
        CL_control_mode_is_active = !CL_control_mode_is_active;
        reset_all_motors();
    }
    
    // See if operator is using the gamepad (if so, disable CL control, and update reference joint position).
    if(proximal_joint_cmds(msg) || distal_joint_cmds(msg))
    {
        D_pad_in_use = true;
        ROS_INFO("D-pad in Use!\n");
    }
    else
    {
        D_pad_in_use = false;
        ROS_INFO("Look, no hands!\n");
    }
    
    // Keep it clean!
    if(filter_duplicate_packets() == 0)
    {
        motor_pub.publish(motor_packet);
    }
}


// TODO:

// -> Use a gamepad btn to cycle throught co-contractions (adjusts I-sense threshold)

// NOTES:
// -> Idle I-sense reading (no I-draw) ~ 580
// -> Define arbitrary ranges for three distinct co-contraction values:
//      * 500
//      * 350
//      * 200




/* TEST_1 */
//proximal joint range (max_btm, max_up):       (2400 +/- 10, 1370 +/- 10)
//distal joint range   (max_left, max_right):   (2878 +/- 10, 1850 +/- 10)


// Can only detect torque values when driving motor

void CL_control()
{
    /* Sanity test 1: "Hold initial position" */ 

    // -> Sample the current joint angle.
    // -> If the joint is perturbed, drive the motors such that the joint is verged to.
    // -> Only use I-sense data to prevent over-loading the motors (identify an absoulte value for this).
    // -> Fitler noise/ transients in I-sense data by ignoring values that massively deveiate from the previous value (hysteresis band).


   // (1): first, make simple P-controller to hold initial position -- initially assuming no tension variation 
   
   // initial_position - current position = err

   // if err >= 0 go CW, else go CCW (correct later if wrong)
   



    /*----------------------------*/
    /* Proximal joint controller */
    /*--------------------------*/

    float P = 2;
    int hysteresis_band = 25;
    
    int proximal_err  = initial_proximal_joint_value  - proximal_joint_value;

    ROS_INFO("Proximal Err: %d\n", proximal_err);

    // If error is within hysteresis band, ignore it.
    if ( (hysteresis_band - abs(proximal_err) > 0) ) 
    {
        proximal_err = 0;
    }


    float corrective_term = P * float(proximal_err);
    
    //drive motors CW
    if (corrective_term < 0) 
    {

        corrective_term = abs(corrective_term + hysteresis_band); //compensate hysteresis offset.

        if(corrective_term >= 255)
        {
            corrective_term = 255; // saturate to 255.
        }

        motor_override(abs(corrective_term), "proximal_motor_A", 0);
        motor_override(abs(corrective_term), "proximal_motor_B", 1);
    }

    //drive proximal motors CCW
    if(corrective_term >= 0)
    {

        corrective_term = abs(corrective_term - hysteresis_band);

        if(corrective_term >= 255)
        {
            corrective_term = 255; // saturate to 255.
        }

        motor_override(abs(corrective_term), "proximal_motor_A", 1);
        motor_override(abs(corrective_term), "proximal_motor_B", 0);
    }
    

    motor_pub.publish(motor_packet);

    // int distal_err    = initial_distal_joint_value    - distal_joint_value;
  

}