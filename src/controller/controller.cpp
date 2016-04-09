#include "ros/ros.h"
#include "quaternion_pd_controller.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller");
    ROS_INFO("Launching node controller.");
    QuaternionPdController controller;


    unsigned int frequency = 10; // Param server
    ros::Rate loop_rate(frequency);
    while (ros::ok())
    {
        ros::spinOnce();
        controller.compute();
        loop_rate.sleep();
    }

    return 0;
}
