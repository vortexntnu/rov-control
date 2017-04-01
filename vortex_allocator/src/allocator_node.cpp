#include "ros/ros.h"

#include "vortex_allocator/allocator_ros.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "allocator");
  ROS_INFO("Launching node.");
  ros::NodeHandle nh;
  Allocator allocator(nh);
  while (ros::ok())
    ros::spinOnce();
  return 0;
}
