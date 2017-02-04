#ifndef CONTROLLER_ROS_H
#define CONTROLLER_ROS_H

#include <Eigen/Dense>

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <dynamic_reconfigure/server.h>

#include <uranus_dp/ControllerConfig.h>
#include "uranus_dp/eigen_typedefs.h"
#include "uranus_dp/control_mode_enum.h"
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
  void configCallback(uranus_dp::ControllerConfig& config, uint32_t level);
  void spin();
private:
  ros::NodeHandle nh;
  ros::Subscriber command_sub;
  ros::Subscriber state_sub;
  ros::Publisher  wrench_pub;
  dynamic_reconfigure::Server<uranus_dp::ControllerConfig> dr_srv;

  ControlMode control_mode;
  int  frequency;
  static const double FORCE_DEADZONE_LIMIT = 0.01;

  State                  *state;
  Setpoints              *setpoints;
  QuaternionPdController *position_hold_controller;

  void initSetpoints();
  void initPositionHoldController();
  bool healthyMessage(const vortex_msgs::PropulsionCommand &msg);
};

#endif
