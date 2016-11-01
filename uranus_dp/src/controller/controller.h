#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "open_loop_controller.h"
#include "quaternion_pd_controller.h"

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
  ros::NodeHandle    nh;
  ros::Subscriber    command_sub;
  ros::Subscriber    state_sub;
  ros::Publisher     pose_pub;
  ros::Publisher     wrench_pub;
  ros::Time          prev_time;

  ControlMode control_mode;
  bool prev_time_valid;
  int  frequency;

  typedef Eigen::Matrix<double,6,1> Vector6d;
  Vector6d pose;
  Vector6d pose_setpoint;
  Vector6d wrench_setpoint;

  typedef std::vector<double> vector;
  vector wrench_command_max;
  vector wrench_command_scaling;
  vector pose_command_rate;

  OpenLoopController     open_loop_controller;
  QuaternionPdController position_hold_controller;

  void updatePoseSetpoints(const vortex_msgs::JoystickMotionCommand &msg);
  void publishPoseSetpoints();
  void updateWrenchSetpoints(const vortex_msgs::JoystickMotionCommand &msg);
  void publishWrenchSetpoints();
  void getParams();
  bool healthyMessage(const vortex_msgs::JoystickMotionCommand &msg);
};

#endif
