#include <Arduino.h>

//#define USE_USBCON 

#include <ros.h>

#include "std_msgs/UInt16MultiArray.h"
#include "std_msgs/UInt16.h"
#include "Wire.h"

  /// Arduino Micro resources...
  //  PWM duty mapping : https://www.theengineeringprojects.com/2017/03/use-arduino-pwm-pins.html
  //  pin-out:           https://www.arduino.cc/en/uploads/Main/ArduinoMicro_Pinout3.png
  //  Wire library help: https://forum.arduino.cc/index.php?topic=391746.0
  //                     


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

  std_msgs::UInt16 proximal_encoder_packet;
  std_msgs::UInt16 distal_encoder_packet;

  ros::Publisher prxml_encdr_pub("proximal_encoder", &proximal_encoder_packet);
  ros::Publisher dstl_encdr_pub("distal_encoder", &distal_encoder_packet);
  
  /* Setup node and it's topics */
  nh.initNode();

  nh.advertise(prxml_encdr_pub);
  nh.advertise(dstl_encdr_pub);

  /* Setup I2C BUS (default I2C pins?) */
  Wire.begin();
  unsigned int raw_data = 0;
  
  
  
  while(1)
  {

    /* Read Proximal encoder on I2C BUS, and then publish */
    raw_data = 0;
    Wire.requestFrom(prxml_encdr_addr, 2); //StartBit + SlaveAddr + ReadBit, wait for Ack, get two dataBytes, send StopCondition.
    raw_data |= (Wire.read()<<8);                 //Store first high-byte
    raw_data |= Wire.read();                      //Store second low-byte
    while(Wire.available() != 0);                 //block until bus is free.

    proximal_encoder_packet.data = raw_data;
    prxml_encdr_pub.publish(&proximal_encoder_packet);

    /* Read Distal encoder on I2C BUS, and then publish */
    raw_data = 0;
    Wire.requestFrom(dstl_encdr_addr, 2); 
    raw_data |= (Wire.read()<<8);
    raw_data |= Wire.read();
    while(Wire.available() != 0);
    
    distal_encoder_packet.data = raw_data;
    dstl_encdr_pub.publish(&distal_encoder_packet);

    /* Service any queued subscriber callbacks */
    nh.spinOnce();

    delay(10);
  }

}


