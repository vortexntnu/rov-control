// #include "integration_filter.h"
#include "simple_estimator.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "estimator");
  ROS_INFO("Launching node estimator.");
  ros::NodeHandle nh;
  SimpleEstimator e;
  // IntegrationFilter estimator;
  // ros::ServiceServer ss = nh.advertiseService("reset_estimator", &IntegrationFilter::reset, &estimator);
  ros::spin();
  return 0;
}
