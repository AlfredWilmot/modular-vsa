#How to setup a gamepad with ROS:
https://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick


#How to setup the joy node after initial installation:
``` 
$ roscore
$ rosparam set joy_node/dev "/dev/input/js0"
$ rosrun joy joy_node
```