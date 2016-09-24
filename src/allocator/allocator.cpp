#include "lagrange_allocator.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "allocator");
  ROS_INFO("Launching node allocator.");
  LagrangeAllocator allocator;
  ros::spin();
  return 0;
}
