#include <Arduino.h>

#define USE_USBCON 

#include <ros.h>

#include "std_msgs/UInt16MultiArray.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/Int16.h"
#include <MCP3221.h>

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
  ros::NodeHandle nh;
  nh.getHardware()->setBaud(115200);
  ros::Subscriber<std_msgs::UInt16MultiArray> sub("segment_motor_cmds", &test_proximal_joint);
  
  std_msgs::UInt16 pub_data;
  pub_data.data = 1;
  ros::Publisher chatter("chatter", &pub_data);


  /* MCP3221 declarations */
  const byte prxml_encdr_addr = 0x4D;
  unsigned long timeNow;
  MCP3221 proximal_encoder(prxml_encdr_addr);

  // PWM pins
  pinMode(EN_A, OUTPUT);
  pinMode(EN_B, OUTPUT);

  // Digital pins
  pinMode(IN_1, OUTPUT);
  pinMode(IN_2, OUTPUT);
  pinMode(IN_3, OUTPUT);
  pinMode(IN_4, OUTPUT);


  /* Setup I2C BUS (default I2C pins?) */
  Wire.begin();
  /* Configure MCP3221 object */
  proximal_encoder.setVref(4096);               // sets voltage reference for the ADC in mV (change as needed)
  proximal_encoder.setVinput(VOLTAGE_INPUT_5V); // sets voltage input type to be measured (change as needed)

  /* Setup node and it's topics */
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(chatter);

///  Serial.begin(115200);

  while(1)
  {
    delay(10);
    pub_data.data = 1;//proximal_encoder.getData();
    delay(10);
    chatter.publish(&pub_data);
    nh.spinOnce();
  }

}

