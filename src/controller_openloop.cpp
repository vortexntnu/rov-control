#include "ros/ros.h"
#include "uranus_dp/ThrusterForces.h"

class ConvertJoystickToThruster
{
public:
    ConvertJoystickToThruster()
    {
        pub = n.advertise<uranus_dp::ThrusterForces>("thruster", 1);
        // sub = n.subscribe("joystick", 1, &ConvertJoystickToThruster::callback, this);
    }

    void callback(const uranus_dp::JoystickUranus& input)
    {
        uranus_dp::ThrusterForces output;
        output.F1 = input.surge;
        output.F2 = input.sway;
        output.F3 = input.heave;
        output.F4 = input.roll;
        output.F5 = input.pitch;
        output.F6 = input.yaw;

        // Blue Robotics T100 thrusters can give max 17.8 Newton thrust both ways
        // (slightly more forward but who cares)
        const double maxThrusterForce = 17.8;
        if (output.F1 >  maxThrusterForce) {output.F1 =  maxThrusterForce;}
        if (output.F1 < -maxThrusterForce) {output.F1 = -maxThrusterForce;}
        if (output.F2 >  maxThrusterForce) {output.F2 =  maxThrusterForce;}
        if (output.F2 < -maxThrusterForce) {output.F2 = -maxThrusterForce;}
        if (output.F3 >  maxThrusterForce) {output.F3 =  maxThrusterForce;}
        if (output.F3 < -maxThrusterForce) {output.F3 = -maxThrusterForce;}
        if (output.F4 >  maxThrusterForce) {output.F4 =  maxThrusterForce;}
        if (output.F4 < -maxThrusterForce) {output.F4 = -maxThrusterForce;}
        if (output.F5 >  maxThrusterForce) {output.F5 =  maxThrusterForce;}
        if (output.F5 < -maxThrusterForce) {output.F5 = -maxThrusterForce;}
        if (output.F6 >  maxThrusterForce) {output.F6 =  maxThrusterForce;}
        if (output.F6 < -maxThrusterForce) {output.F6 = -maxThrusterForce;}

        pub.publish(output);
    }

private:
    ros::NodeHandle n;
    ros::Publisher  pub;
    ros::Subscriber sub;
}; // End of class ConvertJoystickToThruster

int main(int argc, char **argv)
{
    ros::init(argc, argv, "placeholder");

    ConvertJoystickToThruster ConversionObject;

    ros::spin();

    return 0;
}
