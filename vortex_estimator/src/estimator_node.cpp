#include "vortex_estimator/simple_estimator.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "estimator");
  ROS_INFO("Launching node.");
  ros::NodeHandle nh;
  SimpleEstimator estimator;
  while (ros::ok())
    ros::spinOnce();
  return 0;
}
