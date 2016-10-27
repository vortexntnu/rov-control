#ifndef SETPOINT_PROCESSING_H
#define SETPOINT_PROCESSING_H

#include "ros/ros.h"
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/Pose.h"
#include "sensor_msgs/FluidPressure.h"

#include "vortex_msgs/JoystickMotionCommand.h"
#include "uranus_dp/SetControlMode.h"
#include "uranus_dp/ResetIntegrationFilter.h"
#include "uranus_dp/control_mode_enum.h"

class SetpointProcessing
{
public:
  SetpointProcessing();
  void callback(const vortex_msgs::JoystickMotionCommand& msg);
private:
  ros::NodeHandle    nh;
  ros::Subscriber    joystickSub;
  ros::Publisher     wrenchPub;
  ros::Publisher     posePub;
  ros::Publisher     pressurePub;
  ros::ServiceClient modeClient;
  ros::ServiceClient resetClient;

  ControlMode control_mode;

  double max_force_x;
  double max_force_y;
  double max_force_z;
  double max_torque_y;
  double max_torque_z;

  double scaling_force_x;
  double scaling_force_y;
  double scaling_force_z;
  double scaling_torque_y;
  double scaling_torque_z;

  void updateOpenLoop(const vortex_msgs::JoystickMotionCommand& msg);
  void updatePositionHold(const vortex_msgs::JoystickMotionCommand& msg);
  bool healthyMessage(const vortex_msgs::JoystickMotionCommand& msg);
};

#endif
