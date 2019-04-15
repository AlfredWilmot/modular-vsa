cmake_minimum_required(VERSION 2.8.3)
project(sara_8dof_v1)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


find_package(catkin REQUIRED)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sara_8dof_v1
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)


include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)

#find_package(xacro REQUIRED)
#add_custom_target(urdfgen ALL ${CMAKE_CURRENT_SOURCE_DIR}/scripts/make_urdf.sh
#WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
#)