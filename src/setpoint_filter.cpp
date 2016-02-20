#include "ros/ros.h"
#include "std_msgs/String.h"
#include <uranus_dp/JoystickUranus.h>
#include <iostream>
#include <string.h>


class SetpointFilter{
public:
    SetpointFilter(){
        pub = n.advertise<std_msgs::String>("s1", 1);
        sub = n.subscribe("joy_input", 1, &SetpointFilter::callback, this);
    }

    void callback(const uranus_dp::JoystickUranus& input)
    {
    }

private:
    ros::NodeHandle n;
    ros::Publisher  pub;
    ros::Subscriber sub;
};

int main(int argc, char** argv){

    ros::init(argc, argv, "setpoint_filter");

    SetpointFilter f;

    ros::spin();

    return 0;

}
