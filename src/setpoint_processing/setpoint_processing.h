#ifndef SETPOINT_PROCESSING_H
#define SETPOINT_PROCESSING_H

#include "ros/ros.h"
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/Pose.h"
#include "sensor_msgs/FluidPressure.h"

#include "maelstrom_msgs/JoystickMotionCommand.h"
#include "uranus_dp/SetControlMode.h"
#include "uranus_dp/ResetIntegrationFilter.h"
#include "../control_mode_enum.h"

class SetpointProcessing
{
public:
  SetpointProcessing();
  void callback(const maelstrom_msgs::JoystickMotionCommand& msg);
private:
  ros::NodeHandle    nh;
  ros::Subscriber    joystickSub;
  ros::Publisher     wrenchPub;
  ros::Publisher     posePub;
  ros::Publisher     pressurePub;
  ros::ServiceClient modeClient;
  ros::ServiceClient resetClient;

  ControlMode control_mode;

  double depth_setpoint;
  ros::Time depth_setpoint_time;

  static const double MAX_FORCE_X  = 33.5; // [N]  Maximum possible force along x axis
  static const double MAX_FORCE_Y  = 33.5; // [N]  Maximum possible force along y axis
  static const double MAX_FORCE_Z  = 11.4; // [N]  Maximum possible force along z axis
  static const double MAX_TORQUE_X = 2.4;  // [Nm] Maximum possible torque around x axis
  static const double MAX_TORQUE_Z = 3.4;  // [Nm] Maximum possible torque around z axis

  void updateOpenLoop(const maelstrom_msgs::JoystickMotionCommand& msg);
  void updatePositionHold(const maelstrom_msgs::JoystickMotionCommand& msg);
  void updateDepthHold(const maelstrom_msgs::JoystickMotionCommand& msg);
  bool healthyMessage(const maelstrom_msgs::JoystickMotionCommand& msg);
};

#endif
