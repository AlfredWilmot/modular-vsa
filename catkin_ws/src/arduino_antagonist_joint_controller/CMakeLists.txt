cmake_minimum_required(VERSION 2.8.3)
project(arduino_antagonist_joint_controller)

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
add_executable(joystick_msg_parser_node src/joystick_msg_parser.cpp)
target_link_libraries(joystick_msg_parser_node ${catkin_LIBRARIES})