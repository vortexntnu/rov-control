#include "ros/ros.h"
#include "funkycontroller.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller");

    NonlinearQuaternionPidController controller;

    unsigned int frequency = 10; // Get from parameter server sometime in the future
    controller.setFrequency(frequency);

    ros::Rate loop_rate(frequency);
    while (ros::ok())
    {
        ros::spinOnce();
        controller.compute();
        loop_rate.sleep();
    }

    return 0;
}
