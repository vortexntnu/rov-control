#include "ros/ros.h"
#include "std_msgs/String.h"
#include <uranus_dp/JoystickUranus.h>
#include <iostream>
#include <string.h>
#include "geometry_msgs/Twist.h"
#include <joystick/directional_input.h>

// Delivers twist.msg

class SetpointProcessing{
public:
    SetpointProcessing(){
        pub = n.advertise<geometry_msgs::Twist>("temp", 1);
        sub = n.subscribe("joy_input", 1, &SetpointProcessing::callback, this);
    }

    void callback(const joystick::directional_input& input)
    {
        geometry_msgs::Twist output;

        output.linear.x = (input.strafe_X/STRAFE_RANGE)*MAX_LINEAR_X;
        output.linear.y = (input.strafe_Y/STRAFE_RANGE)*MAX_LINEAR_Y;
        output.linear.z = (input.ascend/ASCEND_RANGE)*MAX_LINEAR_Z;
        output.angular.x = 0.0;
        output.angular.y = (input.turn_Y/TURN_RANGE)*MAX_ANGULAR_X;
        output.angular.z = (input.turn_X/TURN_RANGE)*MAX_ANGULAR_Z;

        pub.publish(output);
    }

private:
    ros::NodeHandle n;
    ros::Publisher  pub;
    ros::Subscriber sub;

    static const double MAX_LINEAR_X  = 1.0;
    static const double MAX_LINEAR_Y  = 1.0;
    static const double MAX_LINEAR_Z  = 1.0;
    static const double MAX_ANGULAR_X = 1.0;
    static const double MAX_ANGULAR_Y = 1.0;
    static const double MAX_ANGULAR_Z = 1.0;

    static const int STRAFE_RANGE = (2 << 16);
    static const int TURN_RANGE   = (2 << 16);
    static const int ASCEND_RANGE = (2 << 16);
};

int main(int argc, char** argv){
    ros::init(argc, argv, "setpoint_processing");

    SetpointProcessing f;

    ros::spin();

    return 0;
}
