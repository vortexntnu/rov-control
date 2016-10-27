#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "open_loop_controller.h"
#include "quaternion_pd_controller.h"

#include "uranus_dp/control_mode_enum.h"
#include "uranus_dp/SetControllerGains.h"
#include "uranus_dp/SetControlMode.h"

#include "ros/ros.h"

class Controller
{
public:
  Controller();
  bool setControlMode(uranus_dp::SetControlMode::Request &req, uranus_dp::SetControlMode::Response &resp);
  bool setControllerGains(uranus_dp::SetControllerGains::Request &req, uranus_dp::SetControllerGains::Response &resp);
  void spin();
private:
  ros::NodeHandle nh;
  int frequency;

  ControlMode control_mode;
  OpenLoopController     open_loop_controller;
  QuaternionPdController position_hold_controller;
};

#endif
