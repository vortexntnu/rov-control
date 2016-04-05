#include "open_loop.h"

OpenLoop::OpenLoop()
{
    joySub = n.subscribe("joystick", 1, &OpenLoop::joyCallback, this);
    tauPub = n.advertise<geometry_msgs::Wrench>("controlForces", 1);
}

void OpenLoop::joyCallback(const joystick::Joystick &joy_msg)
{
    geometry_msgs::Wrench tau;
    tau.force.x  = joy_msg.strafe_X;
    tau.force.y  = joy_msg.strafe_X;
    tau.force.z  = joy_msg.ascend;
    tau.torque.x = joy_msg.turn_X;
    tau.torque.y = joy_msg.turn_Y;
    tau.torque.z = 0;

    // Let's limit these values
    const double maxForce  = 10; // [N]  Max force in given direction
    const double maxTorque = 2;  // [Nm] Max torque around given axis
    if (tau.force.x  >  maxForce)  {tau.force.x  =  maxForce;}
    if (tau.force.y  >  maxForce)  {tau.force.y  =  maxForce;}
    if (tau.force.z  >  maxForce)  {tau.force.z  =  maxForce;}
    if (tau.torque.x >  maxTorque) {tau.torque.x =  maxTorque;}
    if (tau.torque.y >  maxTorque) {tau.torque.y =  maxTorque;}
    if (tau.torque.z >  maxTorque) {tau.torque.z =  maxTorque;}
    if (tau.force.x  < -maxForce)  {tau.force.x  = -maxForce;}
    if (tau.force.y  < -maxForce)  {tau.force.y  = -maxForce;}
    if (tau.force.z  < -maxForce)  {tau.force.z  = -maxForce;}
    if (tau.torque.x < -maxTorque) {tau.torque.x = -maxTorque;}
    if (tau.torque.y < -maxTorque) {tau.torque.y = -maxTorque;}
    if (tau.torque.z < -maxTorque) {tau.torque.z = -maxTorque;}

    tauPub.publish(tau);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "openloop");
    ROS_INFO("Launching node open_loop.\n");
    OpenLoop openLoop;
    ros::spin();
    return 0;
}
