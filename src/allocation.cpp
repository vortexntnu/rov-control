#include "ros/ros.h"
#include "lagrange_allocator.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "allocation");
    ROS_INFO("Launching node allocation.\n");
    LagrangeAllocator allocator;
    allocator.setWeights(Eigen::MatrixXd::Identity(6,6));
    ros::spin();
    return 0;
}
