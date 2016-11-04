#include "ros/ros.h"

#include "allocator_ros.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "allocator");
  ROS_INFO("Launching node allocator.");
  ros::NodeHandle nh;
  Allocator allocator(nh);
  ros::spin();
  return 0;
}
