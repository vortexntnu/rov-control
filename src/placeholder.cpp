#include "ros/ros.h"
#include "uranus_dp/JoystickUranus.h"
#include "uranus_dp/ThrusterForces.h"

class ConvertJoystickToThruster
{
public:
    ConvertJoystickToThruster()
    {
        pub = n.advertise<uranus_dp::ThrusterForces>("thruster", 1);
        sub = n.subscribe("joystick", 1, &ConvertJoystickToThruster::callback, this);
    }

    void callback(const uranus_dp::JoystickUranus& input)
    {
        uranus_dp::ThrusterForces output;
        output.forceInNewton_1 = input.surge;
        output.forceInNewton_2 = input.sway;
        output.forceInNewton_3 = input.heave;
        output.forceInNewton_4 = input.roll;
        output.forceInNewton_5 = input.pitch;
        output.forceInNewton_6 = input.yaw;

        // Blue Robotics T100 thrusters can give max 17.8 Newton thrust both ways
        // (slightly more forward but who cares)
        const double maxThrusterForce = 17.8;
        if (output.forceInNewton_1 >  maxThrusterForce) {output.forceInNewton_1 =  maxThrusterForce;}
        if (output.forceInNewton_1 < -maxThrusterForce) {output.forceInNewton_1 = -maxThrusterForce;}
        if (output.forceInNewton_2 >  maxThrusterForce) {output.forceInNewton_2 =  maxThrusterForce;}
        if (output.forceInNewton_2 < -maxThrusterForce) {output.forceInNewton_2 = -maxThrusterForce;}
        if (output.forceInNewton_3 >  maxThrusterForce) {output.forceInNewton_3 =  maxThrusterForce;}
        if (output.forceInNewton_3 < -maxThrusterForce) {output.forceInNewton_3 = -maxThrusterForce;}
        if (output.forceInNewton_4 >  maxThrusterForce) {output.forceInNewton_4 =  maxThrusterForce;}
        if (output.forceInNewton_4 < -maxThrusterForce) {output.forceInNewton_4 = -maxThrusterForce;}
        if (output.forceInNewton_5 >  maxThrusterForce) {output.forceInNewton_5 =  maxThrusterForce;}
        if (output.forceInNewton_5 < -maxThrusterForce) {output.forceInNewton_5 = -maxThrusterForce;}
        if (output.forceInNewton_6 >  maxThrusterForce) {output.forceInNewton_6 =  maxThrusterForce;}
        if (output.forceInNewton_6 < -maxThrusterForce) {output.forceInNewton_6 = -maxThrusterForce;}

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
