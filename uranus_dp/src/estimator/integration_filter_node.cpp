// #include "integration_filter.h"
#include "simple_estimator.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "integration_filter");
  ROS_INFO("Launching node integration_filter.");
  ros::NodeHandle nh;
  SimpleEstimator e;
  // IntegrationFilter estimator;
  // ros::ServiceServer ss = nh.advertiseService("reset_integration_filter", &IntegrationFilter::reset, &estimator);
  ros::spin();
  return 0;
}
