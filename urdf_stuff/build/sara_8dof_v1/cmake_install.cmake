# Install script for directory: /home/alfie/Desktop/Modular-2-DOF-Cable-Driven-Segment/urdf_stuff/src/sara_8dof_v1

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/alfie/Desktop/Modular-2-DOF-Cable-Driven-Segment/urdf_stuff/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/alfie/Desktop/Modular-2-DOF-Cable-Driven-Segment/urdf_stuff/build/sara_8dof_v1/catkin_generated/installspace/sara_8dof_v1.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/sara_8dof_v1/cmake" TYPE FILE FILES
    "/home/alfie/Desktop/Modular-2-DOF-Cable-Driven-Segment/urdf_stuff/build/sara_8dof_v1/catkin_generated/installspace/sara_8dof_v1Config.cmake"
    "/home/alfie/Desktop/Modular-2-DOF-Cable-Driven-Segment/urdf_stuff/build/sara_8dof_v1/catkin_generated/installspace/sara_8dof_v1Config-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/sara_8dof_v1" TYPE FILE FILES "/home/alfie/Desktop/Modular-2-DOF-Cable-Driven-Segment/urdf_stuff/src/sara_8dof_v1/package.xml")
endif()

