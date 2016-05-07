#include "ros/ros.h"
#include "integration_filter.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "state_estimator");
    ROS_INFO("Launching node state_estimator.");

    ros::NodeHandle nh;
    IntegrationFilter estimator;
    ros::ServiceServer ss = nh.advertiseService("reset_state_estimator", &IntegrationFilter::reset, &estimator);
    ros::spin();
    return 0;
}
