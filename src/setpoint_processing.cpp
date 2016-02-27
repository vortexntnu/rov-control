#include "ros/ros.h"
#include "std_msgs/String.h"
#include <uranus_dp/JoystickUranus.h>
#include <iostream>
#include <string.h>
#include "geometry_msgs/Twist.h"


class SetpointProcessing{
public:
    SetpointProcessing(){
        pub = n.advertise<std_msgs::String>("s1", 1);
        sub = n.subscribe("joy_input", 1, &SetpointProcessing::callback, this);
    }

    void callback(const uranus_dp::JoystickUranus& input)
    {
        geometry_msgs::Twist output;
        output.linear.x = input.surge;
        output.linear.y = input.sway;
        output.linear.z = input.heave;
        output.angular.x = input.roll;
        output.angular.y = input.pitch;
        output.angular.z = input.yaw;
    }

private:
    ros::NodeHandle n;
    ros::Publisher  pub;
    ros::Subscriber sub;
};

int main(int argc, char** argv){
    ros::init(argc, argv, "setpoint_processing");

    SetpointProcessing f;

    ros::spin();

    return 0;
}
