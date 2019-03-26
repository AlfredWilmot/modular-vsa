#include <Arduino.h>

#define USE_USBCON 

#include <ros.h>

#include "std_msgs/UInt16MultiArray.h"
#include "std_msgs/UInt16.h"
#include "Wire.h"

  /// Arduino Micro resources...
  //  PWM duty mapping : https://www.theengineeringprojects.com/2017/03/use-arduino-pwm-pins.html
  //  pin-out:           https://www.arduino.cc/en/uploads/Main/ArduinoMicro_Pinout3.png
  //  Wire library help: https://forum.arduino.cc/index.php?topic=391746.0
  //                     

/***********************/
/* PROXIMAL JOINT PINS*/
/*********************/
/* L298N control pins (Set these LO to perform free-running stop) */
const int EN_A_prox = 13;  // (pwm)
const int EN_B_prox = 11;  // (pwm)


/* Pin pairs define motor rotation direction (Set pairs equal, if corresponding EN is HI, to perform hard-stop) */
const int IN_1_prox = 12;
const int IN_2_prox = A0;
const int IN_3_prox = A1;
const int IN_4_prox = A2;

const int EN_A_prox_DUTY = 0;
const int EN_B_prox_DUTY = 0;

/*********************/
/* DISTAL JOINT PINS*/
/*******************/
const int EN_A_dist = A9;
const int EN_B_dist = A10;

const int IN_1_dist = 5;
const int IN_2_dist = 6;
const int IN_3_dist = 7;
const int IN_4_dist = 8;

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
}


void setup() 
{
}


void loop() {

  // Proximal Encoder
  const int prxml_encdr_addr = 77; //0x4D

  // Distal Encoder 
  const int dstl_encdr_addr = 72; //0x48


  /* ROS declarations */
  ros::NodeHandle nh;
  nh.getHardware()->setBaud(115200);
  ros::Subscriber<std_msgs::UInt16MultiArray> sub("segment_motor_cmds", &test_proximal_joint);

  std_msgs::UInt16 proximal_encoder_packet;
  std_msgs::UInt16 distal_encoder_packet;

  ros::Publisher prxml_encdr_pub("proximal_encoder", &proximal_encoder_packet);
  ros::Publisher dstl_encdr_pub("distal_encoder", &distal_encoder_packet);

  /* Setup node and it's topics */
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(prxml_encdr_pub);
  nh.advertise(dstl_encdr_pub);

  /* Setup I2C BUS (default I2C pins?) */
  Wire.begin();

  
  // PWM pins
  pinMode(EN_A_prox, OUTPUT);
  pinMode(EN_B_prox, OUTPUT);

  // Digital pins
  pinMode(IN_1_prox, OUTPUT);
  pinMode(IN_2_prox, OUTPUT);
  pinMode(IN_3_prox, OUTPUT);
  pinMode(IN_4_prox, OUTPUT);

  //Serial.begin(9600);
  unsigned int raw_data = 0;
  
  while(1)
  {

    /* Read Proximal encoder on I2C BUS, and then publish */
    raw_data = 0;
    Wire.requestFrom(prxml_encdr_addr, 2, false); //StartBit + SlaveAddr + ReadBit, wait for Ack, get two dataBytes, send StopCondition.
    raw_data |= (Wire.read()<<8);                 //Store first high-byte
    raw_data |= Wire.read();                      //Store second low-byte
    while(Wire.available() != 0);                 //block until bus is free.

    proximal_encoder_packet.data = raw_data;
    prxml_encdr_pub.publish(&proximal_encoder_packet);

    /* Read Distal encoder on I2C BUS, and then publish */
    raw_data = 0;
    Wire.requestFrom(dstl_encdr_addr, 2, false); 
    raw_data |= (Wire.read()<<8);
    raw_data |= Wire.read();
    while(Wire.available() != 0);
    
    distal_encoder_packet.data = raw_data;
    dstl_encdr_pub.publish(&distal_encoder_packet);

    nh.spinOnce();

    delay(10);
  }

}

