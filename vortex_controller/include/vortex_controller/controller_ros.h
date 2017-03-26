#ifndef CONTROLLER_ROS_H
#define CONTROLLER_ROS_H

#include <Eigen/Dense>

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <dynamic_reconfigure/server.h>

#include <vortex_controller/VortexControllerConfig.h>
#include "vortex/eigen_typedefs.h"
#include "vortex_controller/control_modes.h"
#include "vortex_msgs/PropulsionCommand.h"

#include "state.h"
#include "setpoints.h"
#include "quaternion_pd_controller.h"

class Controller
{
public:
  Controller(ros::NodeHandle nh);
  void commandCallback(const vortex_msgs::PropulsionCommand &msg);
  void stateCallback(const nav_msgs::Odometry &msg);
  void configCallback(vortex_controller::VortexControllerConfig& config, uint32_t level);
  void spin();
private:
  ros::NodeHandle nh;
  ros::Subscriber command_sub;
  ros::Subscriber state_sub;
  ros::Publisher  wrench_pub;
  ros::Publisher  mode_pub;
  dynamic_reconfigure::Server<vortex_controller::VortexControllerConfig> dr_srv;

  ControlMode control_mode;
  int  frequency;
  const double FORCE_DEADZONE_LIMIT = 0.01;

  State                  *state;
  Setpoints              *setpoints;
  QuaternionPdController *position_hold_controller;

  void initSetpoints();
  void initPositionHoldController();
  bool healthyMessage(const vortex_msgs::PropulsionCommand &msg);
  void publishControlMode();
};

#endif
