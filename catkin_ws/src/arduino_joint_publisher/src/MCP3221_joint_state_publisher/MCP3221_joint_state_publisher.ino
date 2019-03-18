
#define USE_USBCON //Avoids USB error w/ Arduino Micro

#include <ros.h>
#include <std_msgs/Float32.h>

#include <MCP3221.h>

/* MCP3221 declarations */
const byte prxml_encdr_addr = 0x4D;
unsigned long timeNow;
MCP3221 proximal_encoder(prxml_encdr_addr);


/* ROS declarations */
ros::NodeHandle nh;
std_msgs::Float32 pub_data;
ros::Publisher chatter("chatter", &pub_data);


void setup() 
{  
  /* Initialize rosnode and chatter topic */
  nh.initNode();
  nh.advertise(chatter);

  /* Publisher will fail to transmit if the serial object is not instantiated with baud 57600*/
  Serial.begin(57600);
    
  /* Setup I2C BUS (default I2C pins?) */
  Wire.begin();
  
  /* Configure MCP3221 object */
  proximal_encoder.setVref(4096);               // sets voltage reference for the ADC in mV (change as needed)
  proximal_encoder.setVinput(VOLTAGE_INPUT_5V); // sets voltage input type to be measured (change as needed)
}

void loop() 
{

  pub_data.data = proximal_encoder.getData();
  chatter.publish(&pub_data);
  nh.spinOnce();
  delay(2);

}
