#include "ros/ros.h"
#include "extended_kalman_filter.h"
#include "integration_filter.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "state_estimator");
    ROS_INFO("Launching node state_estimator.");

    double frequency = 100; // Parameter server
    // ExtendedKalmanFilter estimator(1/frequency);
    IntegrationFilter estimator;

    ros::Rate loop_rate(frequency);
    while (ros::ok())
    {
        ros::spinOnce();
        // estimator.update();
        loop_rate.sleep();
    }
}
