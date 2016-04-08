#include "open_loop.h"

OpenLoop::OpenLoop()
{
    joySub = n.subscribe("joystick", 1, &OpenLoop::joyCallback, this);
    tauPub = n.advertise<geometry_msgs::Wrench>("control_forces", 1);
}

void OpenLoop::joyCallback(const joystick::Joystick &joy_msg)
{
    geometry_msgs::Wrench tau;
    tau.force.x  = joy_msg.strafe_X * normalization * scalingLinear;
    tau.force.y  = joy_msg.strafe_Y * normalization * scalingLinear;
    tau.force.z  = joy_msg.ascend   * normalization * scalingLinear;
    tau.torque.x = 0;
    tau.torque.y = joy_msg.turn_X   * normalization * scalingAngular;
    tau.torque.z = joy_msg.turn_Y   * normalization * scalingAngular;

    tauPub.publish(tau);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "openloop");
    ROS_INFO("Launching node open_loop.");
    OpenLoop openLoop;
    ros::spin();
    return 0;
}
