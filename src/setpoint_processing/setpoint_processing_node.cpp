#include "setpoint_processing.h"

int main(int argc, char** argv){
  ros::init(argc, argv, "setpoint_processing");
  ROS_INFO("Launching node setpoint_processing.");
  SetpointProcessing s;
  ros::spin();
  return 0;
}
