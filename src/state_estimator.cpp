#include "ros/ros.h"
#include "extended_kalman_filter.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "state_estimator");

    double frequency = 100; // Parameter server
    ExtendedKalmanFilter estimator(1/frequency);

    ros::Rate loop_rate(frequency);
    while (ros::ok())
    {
        ros::spinOnce();
        estimator.update();
        loop_rate.sleep();
    }
}
