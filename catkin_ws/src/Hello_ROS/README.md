# Setting-up rosserial_arduino: https://wiki.ros.org/rosserial_arduino/Tutorials/


## (1) Get the necessary packages

`$: ros-kinetic-rosserial-arduino`
`$: ros-kinetic-rosserial-python`


/* TODO LATER: Can create ROS packages for MCU code, and use other IDEs besides Arduino by following this tutorial: https://wiki.ros.org/rosserial_arduino/Tutorials/CMake */
//Invesetigate the mbed, msgs, server, and embeddedLinux variants of rosserial to further expand on this.


In order for the packages to be accessible, need to source devel/setup.bash a catkin workspace */

## (2) Place code below (from here: https://wiki.ros.org/rosserial_arduino/Tutorials/Hello%20World) into an Arduino sketch


```cpp
/*
 * rosserial Publisher Example
 * Prints "hello world!"
 */

// Use the following line if you have a Leonardo or MKR1000 
#define USE_USBCON 

#include <ros.h>
#include <std_msgs/String.h>

ros::NodeHandle nh;

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

char hello[13] = "hello world!";

void setup()
{
  nh.initNode();
  nh.advertise(chatter);
}

void loop()
{
  str_msg.data = hello;
  chatter.publish( &str_msg );
  nh.spinOnce();
  delay(1000);
}
```



/* NOTE: I AM USING `#define USE_USBCON` (with Arduino Micro) as otherwise I get the following error: */
`[ERROR] [1524089506.982801]: Unable to sync with device;`
`possible link problem or link software version mismatch such as hydro rosserial_python with groovy Arduino`


## (3) Compile, Upload as you typically would via Arduino IDE

Once binaries have been uploaded to MCU, run `roscore` in a terminal, and then in another terminal `rosrun rosserial_python serial_node.py <path_to_mcu_port>`

Your MCU port can be found by trawling through your linuc /dev/ directory.





