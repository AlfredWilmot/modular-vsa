#include <Arduino.h>

#define USE_USBCON 

#include <ros.h>

#include "std_msgs/UInt16MultiArray.h"
#include "std_msgs/UInt16.h"
#include "MCP3221.h"

  /// Arduino Micro resources...
  //  PWM duty mapping : https://www.theengineeringprojects.com/2017/03/use-arduino-pwm-pins.html
  //  pin-out:           https://www.arduino.cc/en/uploads/Main/ArduinoMicro_Pinout3.png


/* L298N control pins (Set these LO to perform free-running stop) */
const int EN_A = 13;  // (pwm)
const int EN_B = 11;  // (pwm)


/* Pin pairs define motor rotation direction (Set pairs equal, if corresponding EN is HI, to perform hard-stop) */
const int IN_1 = 12;
const int IN_2 = A0;
const int IN_3 = A1;
const int IN_4 = A2;

const int EN_A_DUTY = 0;
const int EN_B_DUTY = 0;


/* Joystick subscriber callback for controlling motors of one joint */
void test_proximal_joint(const std_msgs::UInt16MultiArray& msg)
{

    int left_motor_dir   = msg.data[0];
    int left_motor_duty  = msg.data[1];
    int right_motor_dir  = msg.data[2];
    int right_motor_duty = msg.data[3];
    


 
      if(right_motor_dir)
      {
        // FWD?
        digitalWrite(IN_1, HIGH);
        digitalWrite(IN_2, LOW);
      }
      else
      {
        //BKWD?
        digitalWrite(IN_1, LOW);
        digitalWrite(IN_2, HIGH);
      }

      analogWrite(EN_A, right_motor_duty);



      if(left_motor_dir)
      {
        // FWD?
        digitalWrite(IN_3, HIGH);
        digitalWrite(IN_4, LOW);
      }
      else
      {
        //BKWD?
        digitalWrite(IN_3, LOW);
        digitalWrite(IN_4, HIGH);
      }

      analogWrite(EN_B, left_motor_duty);
}



void setup() 
{
}


void loop() {

  /* MCP3221 declarations */

  // Proximal Encoder
  const byte prxml_encdr_addr = 0x4D;
  MCP3221 proximal_encoder(prxml_encdr_addr);

  // Distal Encoder 
  const byte dstl_encdr_addr = 0x48;
  MCP3221 distal_encoder(dstl_encdr_addr);


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

  /* Configure MCP3221 objects */
//  proximal_encoder.setVref(4096);               // sets voltage reference for the ADC in mV (change as needed)
//  proximal_encoder.setVinput(VOLTAGE_INPUT_5V); // sets voltage input type to be measured (change as needed)
//  distal_encoder.setVref(4096);               
//  distal_encoder.setVinput(VOLTAGE_INPUT_5V); 

  // PWM pins
  pinMode(EN_A, OUTPUT);
  pinMode(EN_B, OUTPUT);

  // Digital pins
  pinMode(IN_1, OUTPUT);
  pinMode(IN_2, OUTPUT);
  pinMode(IN_3, OUTPUT);
  pinMode(IN_4, OUTPUT);

  //Serial.begin(9600);
  unsigned int raw_data = 0;
  while(1)
  {

    //proximal_encoder_packet.data = proximal_encoder.getData();

    //distal_encoder_packet.data = distal_encoder.getData();

    
    raw_data = 0;
    Wire.requestFrom(77, 2, false); //StartBit + SlaveAddr + ReadBit, wait for Ack, get two dataBytes, send StopCondition.
    raw_data |= (Wire.read()<<8);
    raw_data |= Wire.read();
    //Serial.print("\nProximal Encoder: ");
    //Serial.print(raw_data);
    while(Wire.available() != 0); //block until bus is free.
    proximal_encoder_packet.data = raw_data;
    prxml_encdr_pub.publish(&proximal_encoder_packet);

    raw_data = 0;
    Wire.requestFrom(72, 2, false); //StartBit + SlaveAddr + ReadBit, wait for Ack, get two dataBytes, send StopCondition.
    raw_data |= (Wire.read()<<8);
    raw_data |= Wire.read();
    //Serial.print("\nDistal Encoder: ");
    //Serial.print(raw_data);
    while(Wire.available() != 0);
    
    distal_encoder_packet.data = raw_data;
    dstl_encdr_pub.publish(&distal_encoder_packet);
    
    //prxml_encdr_pub.publish(&proximal_encoder_packet);
    //dstl_encdr_pub.publish(&distal_encoder_packet);

    nh.spinOnce();

    delay(10);
  }

}

