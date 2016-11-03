#include "controller_ros.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "controller");
  ROS_INFO("Launching node controller.");
  ros::NodeHandle nh;
  Controller controller(nh);
  controller.spin();
  return 0;
}
