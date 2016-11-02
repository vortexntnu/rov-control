#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "quaternion_pd_controller.h"

#include "uranus_dp/eigen_typedefs.h"
#include "uranus_dp/control_mode_enum.h"

#include "vortex_msgs/JoystickMotionCommand.h"

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"

#include <Eigen/Dense>

// Dynamic reconfigure includes.
#include <dynamic_reconfigure/server.h>
// Auto-generated from cfg/ directory.
#include <uranus_dp/uranusDpConfig.h>

class Controller
{
public:
  Controller(ros::NodeHandle nh);
  void commandCallback(const vortex_msgs::JoystickMotionCommand &msg);
  void stateCallback(const nav_msgs::Odometry &msg);
  //! Callback function for dynamic reconfigure server.
  // void configCallback(node_example::nodeExampleConfig& config, uint32_t level);
  void configCallback(uranus_dp::uranusDpConfig& config, uint32_t level);
  void spin();
private:
  ros::NodeHandle nh;
  ros::Subscriber command_sub;
  ros::Subscriber state_sub;
  ros::Publisher  wrench_pub;

  //! Dynamic reconfigure server.
  // dynamic_reconfigure::Server<node_example::nodeExampleConfig> dr_srv_;
  dynamic_reconfigure::Server<uranus_dp::uranusDpConfig> dr_srv_;

  ControlMode control_mode;
  int  frequency;

  ros::Time prev_time;
  bool prev_time_valid;

  Eigen::Vector3d    position_state;
  Eigen::Quaterniond orientation_state;
  Eigen::Vector6d    velocity_state;
  Eigen::Vector3d    position_setpoint;
  Eigen::Quaterniond orientation_setpoint;
  Eigen::Vector6d    wrench_setpoint;

  std::vector<double> wrench_command_max;
  std::vector<double> wrench_command_scaling;
  std::vector<double> pose_command_rate;

  QuaternionPdController *position_hold_controller;

  void updateSetpoints(const vortex_msgs::JoystickMotionCommand &msg);
  void getParams();
  bool healthyMessage(const vortex_msgs::JoystickMotionCommand &msg);
};

#endif
