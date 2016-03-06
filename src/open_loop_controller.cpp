#include "ros/ros.h"
#include "geometry_msgs/Wrench.h"
#include "joystick/Joystick.h"

class OpenLoopController
{
public:
    OpenLoopController()
    {
        joySub = n.subscribe("joystick", 1, &OpenLoopController::joyCallback, this);
        tauPub = n.advertise<geometry_msgs::Wrench>("controlForces", 1);
    }

    void joyCallback(const joystick::Joystick &joy_msg)
    {
        geometry_msgs::Wrench tau;
        tau.force.x  = joy_msg.strafe_X;
        tau.force.y  = joy_msg.strafe_X;
        tau.force.z  = joy_msg.ascend;
        tau.torque.x = joy_msg.turn_X;
        tau.torque.y = joy_msg.turn_Y;
        tau.torque.z = 0;

        // Let's limit these values
        const double max = 10;
        if (tau.force.x  >  max) {tau.force.x  =  max;}
        if (tau.force.y  >  max) {tau.force.y  =  max;}
        if (tau.force.z  >  max) {tau.force.z  =  max;}
        if (tau.torque.x >  max) {tau.torque.x =  max;}
        if (tau.torque.y >  max) {tau.torque.y =  max;}
        if (tau.torque.z >  max) {tau.torque.z =  max;}
        if (tau.force.x  < -max) {tau.force.x  = -max;}
        if (tau.force.y  < -max) {tau.force.y  = -max;}
        if (tau.force.z  < -max) {tau.force.z  = -max;}
        if (tau.torque.x < -max) {tau.torque.x = -max;}
        if (tau.torque.y < -max) {tau.torque.y = -max;}
        if (tau.torque.z < -max) {tau.torque.z = -max;}

        tauPub.publish(tau);
    }
private:
    ros::NodeHandle n;
    ros::Publisher  tauPub;
    ros::Subscriber joySub;
}; // End of class OpenLoopController

int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller_openloop");

    OpenLoopController openLoopController;

    ros::spin();

    return 0;
}
