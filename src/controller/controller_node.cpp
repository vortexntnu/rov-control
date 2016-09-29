#include "controller.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "controller");
  ROS_INFO("Launching node controller.");
  ros::NodeHandle nh;
  Controller controller(10); // Run at 10 Hz
  ros::ServiceServer ss1 = nh.advertiseService("set_control_mode", &Controller::setControlMode, &controller);
  ros::ServiceServer ss2 = nh.advertiseService("set_controller_gains", &Controller::setControllerGains, &controller);
  controller.spin();
  return 0;
}
