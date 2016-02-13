#include "ros/ros.h"
#include <uranus_dp/Joystick.h>
#include <uranus_dp/ThrusterForces.h>

class ConvertJoystickToThruster
{
public:
    ConvertJoystickToThruster()
    {
        pub = n.advertise<std_msgs::ThrusterForces>("thruster", 1)
        sub = n.subscribe("joystick", 1, &ConvertJoystickToThruster::callback, this)
    }

    void callback(const std_msgs::Joystick& input)
    {
        // Make sure all inputs are in range (-100, 100)
        // Is it possible to iterate over the Joystick message without losing the names? This is hardly elegant.
        if (input.surge >  100) {input.surge =  100;}
        if (input.surge < -100) {input.surge = -100;}
        if (input.sway  >  100) {input.sway  =  100;}
        if (input.sway  < -100) {input.sway  = -100;}
        if (input.heave >  100) {input.heave =  100;}
        if (input.heave < -100) {input.heave = -100;}
        if (input.roll  >  100) {input.roll  =  100;}
        if (input.roll  < -100) {input.roll  = -100;}
        if (input.pitch >  100) {input.pitch =  100;}
        if (input.pitch < -100) {input.pitch = -100;}
        if (input.yaw   >  100) {input.yaw   =  100;}
        if (input.yaw   < -100) {input.yaw   = -100;}

        // Blue Robotics T100 thrusters can give max 17.8 Newton thrust both ways
        // (slightly more forward but who cares)
        // Scaling factor from (-100, 100) to (-17.8, 17.8) is 500/89
        std_msgs::ThrusterForces output;

        output.forceInNewton_1 = input.surge * (500/89);
        output.forceInNewton_2 = input.sway  * (500/89);
        output.forceInNewton_3 = input.heave * (500/89);
        output.forceInNewton_4 = input.roll  * (500/89);
        output.forceInNewton_5 = input.pitch * (500/89);
        output.forceInNewton_6 = input.yaw   * (500/89);

        pub.publish(output);
    }

private:
    ros::NodeHandle n;
    ros::Publisher  pub;
    ros::Subscriber sub;
} // End of class ConvertJoystickToThruster

int main(int argc, char **argv)
{
    ros::init(argc, argv, "placeholder");

    ConvertJoystickToThruster ConversionObject;

    ros::spin();

    return 0;
}
