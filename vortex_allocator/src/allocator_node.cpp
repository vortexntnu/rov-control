#include "ros/ros.h"

#include "vortex_allocator/allocator_ros.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "allocator");
  ROS_INFO("Launching node.");
  ros::NodeHandle nh;
  Allocator allocator(nh);
  ros::spin();
  return 0;
}
