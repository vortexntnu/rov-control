#ifndef SETPOINT_PROCESSING_H
#define SETPOINT_PROCESSING_H

#include "uranus_dp/control_mode_enum.h"
#include "vortex_msgs/JoystickMotionCommand.h"

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"

#include <Eigen/Dense>

class SetpointProcessing
{
public:
  SetpointProcessing();
  void commandCallback(const vortex_msgs::JoystickMotionCommand &msg);
  void stateCallback(const nav_msgs::Odometry &msg);
private:
  ros::NodeHandle    nh;
  ros::Subscriber    command_sub;
  ros::Subscriber    state_sub;
  ros::Publisher     pose_pub;
  ros::Publisher     wrench_pub;
  ros::ServiceClient mode_client;
  ros::ServiceClient reset_client;
  ros::Time          prev_time;

  ControlMode control_mode;
  bool prev_time_valid;

  typedef Eigen::Matrix<double,6,1> Vector6d;
  Vector6d pose;
  Vector6d pose_setpoint;
  Vector6d wrench_setpoint;

  typedef std::vector<double> vector;
  vector wrench_command_max;
  vector wrench_command_scaling;
  vector pose_command_rate;

  void updatePoseSetpoints(const vortex_msgs::JoystickMotionCommand &msg);
  void publishPoseSetpoints();
  void updateWrenchSetpoints(const vortex_msgs::JoystickMotionCommand &msg);
  void publishWrenchSetpoints();
  void getParams();
  bool healthyMessage(const vortex_msgs::JoystickMotionCommand &msg);
};

#endif
