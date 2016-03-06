#include "ros/ros.h"
#include "lagrange_allocator.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "allocation");
    LagrangeAllocator allocator;
    ros::spin();
    return 0;
}
