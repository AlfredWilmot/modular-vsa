#TODO: Improve!

Checkout this (https://wiki.ros.org/rosserial) documentation to figure out ways of improving functionality (already quite good for single encoder).

In particular, using rosserial_server (instead of rosserial_python), to minimize latency (https://wiki.ros.org/rosserial_server).

Mainly, using CMake to side-step using Arduino IDE, and incorporating code into catkin_ws (https://wiki.ros.org/rosserial_arduino/Tutorials/CMake); so it can be run similar to a typical ros package.



Can also use rosserial for STM-boards for further imporved functionality! But how much better?! (https://wiki.ros.org/rosserial_mbed/Tutorials/rosserial_mbed%20Setup)
