#ifndef OPEN_LOOP_CONTROLLER_H
#define OPEN_LOOP_CONTROLLER_H

#include "ros/ros.h"
#include "geometry_msgs/Wrench.h"
#include "joystick/Joystick.h"

class OpenLoopController
{
public:
    OpenLoopController();
    void joyCallback(const joystick::Joystick &joy_msg);
private:
    ros::NodeHandle n;
    ros::Publisher  tauPub;
    ros::Subscriber joySub;
};

#endif
