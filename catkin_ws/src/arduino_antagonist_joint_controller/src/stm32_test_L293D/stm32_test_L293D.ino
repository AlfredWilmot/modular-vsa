#include <Arduino.h>

//#define USE_USBCON 

#include <ros.h>

#include "std_msgs/UInt16MultiArray.h"
#include "std_msgs/UInt16.h"
//#include "Wire.h"

  /// Arduino Micro resources...
  //  PWM duty mapping : https://www.theengineeringprojects.com/2017/03/use-arduino-pwm-pins.html
  //  pin-out:           https://www.arduino.cc/en/uploads/Main/ArduinoMicro_Pinout3.png
  //  Wire library help: https://forum.arduino.cc/index.php?topic=391746.0
  //                     




/***********************/
/* PROXIMAL JOINT PINS*/
/*********************/
/* L298N control pins (Set these LO to perform free-running stop) */
const int EN_A_prox = PB9;  // (pwm)
const int EN_B_prox = PB8;  // (pwm)


/* Pin pairs define motor rotation direction (Set pairs equal, if corresponding EN is HI, to perform hard-stop) */
const int IN_1_prox = PB3;
const int IN_2_prox = PB4;
const int IN_3_prox = PA15;
const int IN_4_prox = PA10;

const int EN_A_prox_DUTY = 0;
const int EN_B_prox_DUTY = 0;

/*********************/
/* DISTAL JOINT PINS*/
/*******************/
const int EN_A_dist = PB7;
const int EN_B_dist = PB6;

const int IN_1_dist = PA11;
const int IN_2_dist = PA9;
const int IN_3_dist = PA9;
const int IN_4_dist = PA8;

const int EN_A_dist_DUTY = 0;
const int EN_B_dist_DUTY = 0;

/* Joystick subscriber callback for controlling motors of one joint */
void test_proximal_joint(const std_msgs::UInt16MultiArray& msg)
{

    int left_motor_dir   = msg.data[0];
    int left_motor_duty  = msg.data[1];
    int right_motor_dir  = msg.data[2];
    int right_motor_duty = msg.data[3];
    int toggle_joint     = msg.data[4];



    if(toggle_joint)
    {
      /* Disable the other motor pair */
      analogWrite(EN_A_prox, 0);
      analogWrite(EN_B_prox, 0);
      
      if(right_motor_dir)
      {
        digitalWrite(IN_1_dist, HIGH);
        digitalWrite(IN_2_dist, LOW);
      }
      else
      {
        digitalWrite(IN_1_dist, LOW);
        digitalWrite(IN_2_dist, HIGH);
      }

      analogWrite(EN_A_dist, right_motor_duty);
      
      
      
      if(left_motor_dir)
      {
        digitalWrite(IN_3_dist, HIGH);
        digitalWrite(IN_4_dist, LOW);
      }
      else
      {
        digitalWrite(IN_3_dist, LOW);
        digitalWrite(IN_4_dist, HIGH);
      }
      
      analogWrite(EN_B_dist, left_motor_duty);    
        
    }
    else
    {
      /* Disable the other motor pair */
      analogWrite(EN_A_dist, 0);
      analogWrite(EN_B_dist, 0);
      
      if(right_motor_dir)
      {
        // FWD?
        digitalWrite(IN_1_prox, HIGH);
        digitalWrite(IN_2_prox, LOW);
      }
      else
      {
        //BKWD?
        digitalWrite(IN_1_prox, LOW);
        digitalWrite(IN_2_prox, HIGH);
      }

      analogWrite(EN_A_prox, right_motor_duty);



      if(left_motor_dir)
      {
        // FWD?
        digitalWrite(IN_3_prox, HIGH);
        digitalWrite(IN_4_prox, LOW);
      }
      else
      {
        //BKWD?
        digitalWrite(IN_3_prox, LOW);
        digitalWrite(IN_4_prox, HIGH);
      }

      analogWrite(EN_B_prox, left_motor_duty);
    }


  //DEBUG 
    


}


void setup() 
{
}


void loop() {



  /* ROS declarations */
  ros::NodeHandle nh;
  nh.getHardware()->setBaud(115200);
  ros::Subscriber<std_msgs::UInt16MultiArray> sub("segment_motor_cmds", &test_proximal_joint);
  

  /* Setup node and it's topics */
  nh.initNode();
  nh.subscribe(sub);


  // PWM pinschatter
  pinMode(EN_A_prox, OUTPUT);
  pinMode(EN_B_prox, OUTPUT);

  // Digital pins
  pinMode(IN_1_prox, OUTPUT);
  pinMode(IN_2_prox, OUTPUT);
  pinMode(IN_3_prox, OUTPUT);
  pinMode(IN_4_prox, OUTPUT);
  
  while(1)
  {
    
    nh.spinOnce();

    delay(10);
  }

}


