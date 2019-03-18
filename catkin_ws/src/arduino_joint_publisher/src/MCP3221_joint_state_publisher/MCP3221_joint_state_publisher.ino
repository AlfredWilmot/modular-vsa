#include <ros.h>
#include <std_msgs/String.h>

#include <MCP3221.h>


/* MCP3221 declarations */
const byte prxml_encdr_addr = 0x4D;
unsigned long timeNow;
MCP3221 proximal_encoder(prxml_encdr_addr);


/* ROS declarations */
ros::NodeHandle nh;
std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);
char hello[13] = "hello world!";


void setup() 
{
  nh.initNode();
  nh.advertise(chatter);
  
  Serial.begin(9600);
  Wire.begin();
  Serial.print(F("\n\nserial is open\n\n"));
  proximal_encoder.setVref(4096);                            // sets voltage reference for the ADC in mV (change as needed)
  proximal_encoder.setVinput(VOLTAGE_INPUT_5V);              // sets voltage input type to be measured (change as needed)
  timeNow = millis();
}

void loop() 
{
  if (millis() - timeNow >= 10) 
  {
    Serial.print(F("reading:\t"));
    Serial.print(proximal_encoder.getData());
    Serial.print(F("\n\n"));  
    timeNow = millis();

    str_msg.data = hello;
    chatter.publish( &str_msg );
    nh.spinOnce();
    
  }
}
