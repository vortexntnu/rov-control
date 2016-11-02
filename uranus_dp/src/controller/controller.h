#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "quaternion_pd_controller.h"

#include "uranus_dp/eigen_typedefs.h"
#include "uranus_dp/control_mode_enum.h"
#include "uranus_dp/SetControllerGains.h"

#include "vortex_msgs/JoystickMotionCommand.h"

#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Wrench.h"
#include "nav_msgs/Odometry.h"

#include <tf/transform_datatypes.h>
#include <eigen_conversions/eigen_msg.h>

#include <Eigen/Dense>

class Controller
{
public:
  Controller();
  bool setControllerGains(uranus_dp::SetControllerGains::Request &req, uranus_dp::SetControllerGains::Response &resp);
  void commandCallback(const vortex_msgs::JoystickMotionCommand &msg);
  void stateCallback(const nav_msgs::Odometry &msg);
  void spin();
private:
  ros::NodeHandle nh;
  ros::Subscriber command_sub;
  ros::Subscriber state_sub;
  ros::Publisher  wrench_pub;

  ControlMode control_mode;
  int  frequency;

  ros::Time prev_time;
  bool prev_time_valid;

  // TODO: Consider a typedef Vector6d or something
  Eigen::Vector3d    position_state;
  Eigen::Quaterniond orientation_state;
  Eigen::Vector6d    velocity_state;
  Eigen::Vector3d    position_setpoint;
  Eigen::Quaterniond orientation_setpoint;
  Eigen::Vector6d    wrench_setpoint;

  typedef std::vector<double> vector;
  vector wrench_command_max;
  vector wrench_command_scaling;
  vector pose_command_rate;

  QuaternionPdController *position_hold_controller;

  void updateSetpoints(const vortex_msgs::JoystickMotionCommand &msg);
  void getParams();
  bool healthyMessage(const vortex_msgs::JoystickMotionCommand &msg);
};

#endif
