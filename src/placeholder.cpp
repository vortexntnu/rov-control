#include "ros/ros.h"
#include "../msg"
#include <sstream>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "placeholder");
    ros::NodeHandle n;

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        // read a Joystick message
        // create a ThrusterForces message
        // send the ThrusterForces message
    }
}