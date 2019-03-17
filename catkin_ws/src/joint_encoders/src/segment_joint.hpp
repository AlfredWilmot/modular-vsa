#ifndef __SEGMENT_JOINT__
#define __SEGMENT_JOINT__

    #include <iostream>
    #include <stdio.h>
    #include <cmath>
    #include <string>

    #include "ros/ros.h"
    #include "std_msgs/Float64.h"

    #include "./MCP3221/MCP3221.h"
    
    class segment_joint
    {
    private:
        const byte I2C_slave_addr;

    public:
        segment_joint(/* args */);
        ~segment_joint();
    };
    
    segment_joint::segment_joint(/* args */)
    {
    }
    
    segment_joint::~segment_joint()
    {
    }
    



#endif