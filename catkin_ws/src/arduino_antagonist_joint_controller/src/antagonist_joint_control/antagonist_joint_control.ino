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

/*******************************************************/
/* Define digital IO pins being used on Arduino mirco */
/*****************************************************/


// Motor direction pins 

const int proximal_motor_1a = 11;//5;
const int proximal_motor_1b = 12;//6;
const int proximal_motor_2a = 7;//9;
const int proximal_motor_2b = 8;//10; 

const int distal_motor_1a = 9;//7;
const int distal_motor_1b = 10;//8;
const int distal_motor_2a = 6;//12;
const int distal_motor_2b = 5;//11;

const int Enable_proximal_motor_1 = 23;
const int Enable_proximal_motor_2 = 22;
const int Enable_distal_motor_1   = 21;
const int Enable_distal_motor_2   = 20;


// I-sense ADC pins 
const int I_sense_1 = A0;
const int I_sense_2 = A1;
const int I_sense_3 = A2;
const int I_sense_4 = A3;

/* Joystick subscriber callback for controlling motors of one joint */
void test_proximal_joint(const std_msgs::UInt16MultiArray& msg)
{
  digitalWrite(proximal_motor_1a, msg.data[0]);
  digitalWrite(proximal_motor_1b, msg.data[1]);
  digitalWrite(proximal_motor_2a, msg.data[3]);
  digitalWrite(proximal_motor_2b, msg.data[2]);
//
  digitalWrite(distal_motor_1a, msg.data[5]);
  digitalWrite(distal_motor_1b, msg.data[4]);
  digitalWrite(distal_motor_2a, msg.data[7]);
  digitalWrite(distal_motor_2b, msg.data[6]);

  analogWrite(Enable_proximal_motor_1, msg.data[8]);
  analogWrite(Enable_proximal_motor_2, msg.data[9]);
  analogWrite(Enable_distal_motor_1,   msg.data[10]);
  analogWrite(Enable_distal_motor_2,   msg.data[11]);

}


void setup() 
{
}


void loop() {

  
  // Proximal Encoder
  const int prxml_encdr_addr = 77; //0x4D

  // Distal Encoder 
  //const int dstl_encdr_addr = 72; //0x48
  const int dstl_encdr_addr = 79; //0x4F

  /* ROS declarations */
  ros::NodeHandle nh;
  nh.getHardware()->setBaud(115200);
  ros::Subscriber<std_msgs::UInt16MultiArray> sub("segment_motor_cmds", &test_proximal_joint);

  std_msgs::UInt16 proximal_encoder_packet;
  std_msgs::UInt16 distal_encoder_packet;

  ros::Publisher prxml_encdr_pub("proximal_encoder", &proximal_encoder_packet);
  ros::Publisher dstl_encdr_pub("distal_encoder", &distal_encoder_packet);

  std_msgs::UInt16 I_sense_1_packet;
  std_msgs::UInt16 I_sense_2_packet;
  std_msgs::UInt16 I_sense_3_packet;
  std_msgs::UInt16 I_sense_4_packet;
  
  ros::Publisher I_sense_pub_1("I_sense_1", &I_sense_1_packet);
  ros::Publisher I_sense_pub_2("I_sense_2", &I_sense_2_packet);
  ros::Publisher I_sense_pub_3("I_sense_3", &I_sense_3_packet);
  ros::Publisher I_sense_pub_4("I_sense_4", &I_sense_4_packet);
  
  /* Setup node and it's topics */
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(prxml_encdr_pub);
  nh.advertise(dstl_encdr_pub);

  nh.advertise(I_sense_pub_1);
  nh.advertise(I_sense_pub_2);
  nh.advertise(I_sense_pub_3);
  nh.advertise(I_sense_pub_4);

  /* Setup I2C BUS (default I2C pins?) */
  Wire.begin();
  unsigned int raw_data = 0;
  

  // Digital pins
  pinMode(proximal_motor_1a, OUTPUT);
  pinMode(proximal_motor_1b, OUTPUT);
  pinMode(proximal_motor_2a, OUTPUT);
  pinMode(proximal_motor_2b, OUTPUT);

  pinMode(distal_motor_1a, OUTPUT);
  pinMode(distal_motor_1b, OUTPUT);
  pinMode(distal_motor_2a, OUTPUT);
  pinMode(distal_motor_2b, OUTPUT);
  
  
  pinMode(Enable_proximal_motor_1, OUTPUT);
  pinMode(Enable_proximal_motor_2, OUTPUT);
  pinMode(Enable_distal_motor_1,   OUTPUT);
  pinMode(Enable_distal_motor_2,   OUTPUT);


  // Set all digital pin outputs to LOW to start with
  digitalWrite(proximal_motor_1a, LOW);
  digitalWrite(proximal_motor_1b, LOW);
  digitalWrite(proximal_motor_2a, LOW);
  digitalWrite(proximal_motor_2b, LOW);

  digitalWrite(distal_motor_1a, LOW);
  digitalWrite(distal_motor_1b, LOW);
  digitalWrite(distal_motor_2a, LOW);
  digitalWrite(distal_motor_2b, LOW);


  
  
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



    I_sense_1_packet.data = analogRead(I_sense_1); //12mOhm shunt 
    I_sense_2_packet.data = analogRead(I_sense_2); //25mOhm shunt
    I_sense_3_packet.data = analogRead(I_sense_3); //12mOhn shunt
    I_sense_4_packet.data = analogRead(I_sense_4); //12mOhm shunt
    
    I_sense_pub_1.publish(&I_sense_1_packet);
    I_sense_pub_2.publish(&I_sense_2_packet);
    I_sense_pub_3.publish(&I_sense_3_packet);
    I_sense_pub_4.publish(&I_sense_4_packet);

    
  
    /* Service any queued subscriber callbacks */
    nh.spinOnce();

    delay(10);
  }

}

