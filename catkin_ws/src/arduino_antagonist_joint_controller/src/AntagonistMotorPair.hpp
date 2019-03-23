// #ifndef __ANTAGONISTMOTORPAIR__
// #define __ANTAGONISTMOTORPAIR__


// #include "ros/ros.h"

// #include "sensor_msgs/Joy.h"
// #include "std_msgs/Int8.h"
// #include "std_msgs/Int8MultiArray.h"

// #include <iostream>




// #define CW  0
// #define CCW 1

// class AntagonistMotorPair
// {
// private:

//     /* Packet that stores the motor characteristic ararys for publishing to the topic */
//     std_msgs::Int8 motor_packet[2][2];

//     //static bool node_not_spawned;

// public:
//     AntagonistMotorPair(/* args */);
//     ~AntagonistMotorPair();

//     void set_agonist_motor_duty(int8_t);        // Agonist motor is the one on the same side as it's corresponding joint's magnet.
//     void set_antagonist_motor_duty(int8_t);

//     void set_agonist_motor_dir(bool);
//     void set_antagonist_motor_dir(bool);

// };


// //AntagonistMotorPair::node_not_spawned = true;


// AntagonistMotorPair::AntagonistMotorPair(/* args */)
// {
//     if(0)//node_not_spawned)
//     {
//         /* ROS Node Premable */
//         ros::NodeHandle nh;

//         /* Publish filtered joy-msg into something simple array for the MCU */
//         ros::Publisher motor_pub = nh.advertise<std_msgs::Int8MultiArray>("segment_motor_cmds", 1); //vector of 8-bit integers 

//        // node_not_spawned = false;
//     }
// }

// AntagonistMotorPair::~AntagonistMotorPair()
// {
// }




// #endif