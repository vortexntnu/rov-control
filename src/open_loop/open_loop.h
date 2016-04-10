#ifndef OPEN_LOOP_H
#define OPEN_LOOP_H

#include "ros/ros.h"
#include "geometry_msgs/Wrench.h"
#include "joystick/Joystick.h"

class OpenLoop
{
public:
    OpenLoop();
    void joyCallback(const joystick::Joystick &joy_msg);
private:
    ros::NodeHandle n;
    ros::Publisher  tauPub;
    ros::Subscriber joySub;

    static const double NORMALIZATION  = 0.000030517578125; // Scale inputs down to [-1, 1]
    static const double SCALING_LIN    = 10;                // [N]  Max force in given direction
    static const double SCALING_ANG    = 2;                 // [Nm] Max torque around given axis
};

#endif
