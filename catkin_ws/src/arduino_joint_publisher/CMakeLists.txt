cmake_minimum_required(VERSION 2.8.3)
project(arduino_joint_publisher)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#Adding user-made libraries (local to package)
#add_library(my_joy_lib
#  src/rs232.h
#  src/rs232.c
#  src/Encoder_Class.hpp
#  )

#Creating joystick subscriber exe and then linking it with the relevant user-made and base libraries 
add_executable(my_joy_node src/Test_Xbox_gamepad/joy_subscriber.cpp)
#target_link_libraries(my_joy_node ${catkin_LIBRARIES} my_joy_lib)