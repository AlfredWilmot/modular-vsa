# Install script for directory: /home/alfie/Desktop/Modular-2-DOF-Cable-Driven-Segment/urdf_stuff/src/segment_v_2_3b

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/alfie/Desktop/Modular-2-DOF-Cable-Driven-Segment/urdf_stuff/build/segment_v_2_3b/catkin_generated/installspace/segment_v_2_3b.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/segment_v_2_3b/cmake" TYPE FILE FILES
    "/home/alfie/Desktop/Modular-2-DOF-Cable-Driven-Segment/urdf_stuff/build/segment_v_2_3b/catkin_generated/installspace/segment_v_2_3bConfig.cmake"
    "/home/alfie/Desktop/Modular-2-DOF-Cable-Driven-Segment/urdf_stuff/build/segment_v_2_3b/catkin_generated/installspace/segment_v_2_3bConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/segment_v_2_3b" TYPE FILE FILES "/home/alfie/Desktop/Modular-2-DOF-Cable-Driven-Segment/urdf_stuff/src/segment_v_2_3b/package.xml")
endif()
