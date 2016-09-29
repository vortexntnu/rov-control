#include "controller.h"

Controller::Controller(unsigned int f)
{
  frequency = f;
  control_mode = ControlModes::OPEN_LOOP;
  position_hold_controller.disable();
  depth_hold_controller.disable();
  open_loop_controller.enable();
}

bool Controller::setControlMode(uranus_dp::SetControlMode::Request &req, uranus_dp::SetControlMode::Response &resp)
{
  ControlMode new_control_mode = static_cast<ControlMode>(req.mode);
  if (new_control_mode != control_mode)
  {
    control_mode = new_control_mode;
    switch (control_mode)
    {
      case ControlModes::OPEN_LOOP:
      ROS_INFO("Changing control mode to OPEN LOOP.");
      position_hold_controller.disable();
      depth_hold_controller.disable();
      open_loop_controller.enable();
      break;

      case ControlModes::POSITION_HOLD:
      ROS_INFO("Changing control mode to POSITION HOLD.");
      open_loop_controller.disable();
      depth_hold_controller.disable();
      position_hold_controller.enable();
      break;

      case ControlModes::DEPTH_HOLD:
      ROS_INFO("Changing control mode to DEPTH HOLD.");
      open_loop_controller.disable();
      position_hold_controller.disable();
      depth_hold_controller.enable();
      break;

      default:
      ROS_WARN("Invalid control mode set.");
      break;
    }
  }
  else
  {
    ROS_INFO("Attempt to set already active control mode, ignoring.");
  }
  return true;
}

bool Controller::setControllerGains(uranus_dp::SetControllerGains::Request &req, uranus_dp::SetControllerGains::Response &resp)
{
  ROS_INFO_STREAM("Setting new gains: a = " << req.a << ", b = " << req.b << ", c = " << req.c << ".");
  position_hold_controller.setGains(req.a, req.b, req.c);
}

void Controller::spin()
{
  ros::Rate rate(frequency);
  while (ros::ok())
  {
    ros::spinOnce();
    position_hold_controller.compute();
    depth_hold_controller.compute();
    rate.sleep();
  }
}
