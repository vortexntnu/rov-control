// #include "integration_filter.h"
#include "simple_estimator.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "estimator");
  ROS_INFO("Launching node estimator.");
  ros::NodeHandle nh;
  SimpleEstimator e;
  ros::spin();
  return 0;
}
