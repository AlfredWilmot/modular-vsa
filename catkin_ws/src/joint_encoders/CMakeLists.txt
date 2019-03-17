cmake_minimum_required(VERSION 2.8.3)
project(joint_encoders)

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
add_library(segment_joint_lib
  src/publish_segment_joint_values.cpp
  src/segment_joint.hpp
  /Ard
  src/MCP3221/MCP3221.h
  )

#Creating encoder exe and then linking it with the relevant user-made and base libraries 
add_executable(segment_joint_node src/publish_segment_joint_values.cpp)
target_link_libraries(segment_joint_node ${catkin_LIBRARIES} segment_joint_lib)