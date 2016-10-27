#include "setpoint_processing.h"

SetpointProcessing::SetpointProcessing()
{
  joystickSub = nh.subscribe("joystick_motion_command", 10, &SetpointProcessing::callback, this);
  wrenchPub   = nh.advertise<geometry_msgs::Wrench>("wrench_setpoints", 10);
  posePub     = nh.advertise<geometry_msgs::Pose>("pose_setpoints", 10);
  modeClient  = nh.serviceClient<uranus_dp::SetControlMode>("set_control_mode");
  resetClient = nh.serviceClient<uranus_dp::ResetIntegrationFilter>("reset_integration_filter");

  control_mode = ControlModes::OPEN_LOOP;

  // Read maximum forces/torques from parameter server
  if (!nh.getParam("/actuation/max/force/x", max_force_x))
    ROS_FATAL("Failed to read parameter max_force_x.");
  if (!nh.getParam("/actuation/max/force/y", max_force_y))
    ROS_FATAL("Failed to read parameter max_force_y.");
  if (!nh.getParam("/actuation/max/force/z", max_force_z))
    ROS_FATAL("Failed to read parameter max_force_z.");
  if (!nh.getParam("/actuation/max/torque/y", max_torque_y))
    ROS_FATAL("Failed to read parameter max_torque_y.");
  if (!nh.getParam("/actuation/max/torque/z", max_torque_z))
    ROS_FATAL("Failed to read parameter max_torque_z.");

  // Read force/torque scaling factors from parameter server
  if (!nh.getParam("/actuation/scaling/force/x", scaling_force_x))
    ROS_FATAL("Failed to read parameter scaling_force_x.");
  if (!nh.getParam("/actuation/scaling/force/y", scaling_force_y))
    ROS_FATAL("Failed to read parameter scaling_force_y.");
  if (!nh.getParam("/actuation/scaling/force/z", scaling_force_z))
    ROS_FATAL("Failed to read parameter scaling_force_z.");
  if (!nh.getParam("/actuation/scaling/torque/y", scaling_torque_y))
    ROS_FATAL("Failed to read parameter scaling_torque_y.");
  if (!nh.getParam("/actuation/scaling/torque/z", scaling_torque_z))
    ROS_FATAL("Failed to read parameter scaling_torque_z.");
}

void SetpointProcessing::callback(const vortex_msgs::JoystickMotionCommand& msg)
{
  if (!healthyMessage(msg))
  {
    ROS_WARN("setpoint_processing: Joystick motion command message out of range, ignoring...");
    return;
  }

  if (msg.control_mode != control_mode)
  {
    control_mode = static_cast<ControlMode>(msg.control_mode);
    if (control_mode == ControlModes::POSITION_HOLD)
    {
      // Reset estimator when mode changes to position hold
      uranus_dp::ResetIntegrationFilter reset_srv;
      resetClient.call(reset_srv);
    }

    uranus_dp::SetControlMode srv;
    srv.request.mode = control_mode;
    if (!modeClient.call(srv))
      ROS_ERROR_STREAM("Failed to call service set_control_mode. New mode " << control_mode << " not activated.");
  }

  switch (control_mode)
  {
    case ControlModes::OPEN_LOOP:
    updateOpenLoop(msg);
    break;

    case ControlModes::POSITION_HOLD:
    updatePositionHold(msg);
    break;

    default:
    ROS_WARN("setpoint_processing: Default control mode switch case reached.");
  }
}

void SetpointProcessing::updateOpenLoop(const vortex_msgs::JoystickMotionCommand& msg)
{
  ROS_INFO("setpoint_processing: Sending OPEN_LOOP setpoints.");
  geometry_msgs::Wrench open_loop_msg;
  open_loop_msg.force.x  = scaling_force_x * max_force_x * msg.forward;
  open_loop_msg.force.y  = scaling_force_y * max_force_y * msg.right;
  open_loop_msg.force.z  = scaling_force_z * max_force_z * msg.down;
  open_loop_msg.torque.x = 0;
  open_loop_msg.torque.y = scaling_torque_y * max_torque_y * msg.tilt_up;
  open_loop_msg.torque.z = scaling_torque_z * max_torque_z * msg.turn_right;
  wrenchPub.publish(open_loop_msg);
}

void SetpointProcessing::updatePositionHold(const vortex_msgs::JoystickMotionCommand& msg)
{
  ROS_INFO("setpoint_processing: Sending POSITION_HOLD setpoints.");
  // Todo: Actually populate the pose message with values
  geometry_msgs::Pose position_hold_msg;
  position_hold_msg.position.x    = msg.forward;
  position_hold_msg.position.y    = msg.right;
  position_hold_msg.position.z    = msg.down;
  position_hold_msg.orientation.x = 0;
  position_hold_msg.orientation.y = 0;
  position_hold_msg.orientation.z = 0;
  position_hold_msg.orientation.w = 1;
  posePub.publish(position_hold_msg);
}

bool SetpointProcessing::healthyMessage(const vortex_msgs::JoystickMotionCommand& msg)
{
  if (abs(msg.forward) > 1)
  {
    ROS_WARN("setpoint_processing: Forward motion command out of range");
    return false;
  }
  if (abs(msg.right) > 1)
  {
    ROS_WARN("setpoint_processing: Right motion command out of range");
    return false;
  }
  if (abs(msg.down) > 1)
  {
    ROS_WARN("setpoint_processing: Down motion command out of range.");
    return false;
  }
  if (abs(msg.tilt_up) > 1)
  {
    ROS_WARN("setpoint_processing: Tilt up motion command out of range");
    return false;
  }
  if (abs(msg.turn_right) > 1)
  {
    ROS_WARN("setpoint_processing: Turn right motion command out of range");
    return false;
  }

  bool validControlMode = (msg.control_mode == ControlModes::OPEN_LOOP || msg.control_mode == ControlModes::POSITION_HOLD);
  if (!validControlMode)
  {
    ROS_WARN("setpoint_processing: Invalid control mode.");
    return false;
  }
  return true;
}
