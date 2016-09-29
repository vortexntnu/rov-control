#include "setpoint_processing.h"

SetpointProcessing::SetpointProcessing()
{
  joystickSub = nh.subscribe("joystick_motion_command", 10, &SetpointProcessing::callback, this);
  wrenchPub   = nh.advertise<geometry_msgs::Wrench>("wrench_setpoints", 10);
  posePub     = nh.advertise<geometry_msgs::Pose>("pose_setpoints", 10);
  modeClient  = nh.serviceClient<uranus_dp::SetControlMode>("set_control_mode");
  resetClient = nh.serviceClient<uranus_dp::ResetIntegrationFilter>("reset_integration_filter");

  control_mode = ControlModes::OPEN_LOOP;
  depth_setpoint = 0.5; // [m]
}

void SetpointProcessing::callback(const maelstrom_msgs::JoystickMotionCommand& msg)
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

    case ControlModes::DEPTH_HOLD:
    updateDepthHold(msg);
    break;

    default:
    ROS_WARN("setpoint_processing: Default control mode switch case reached.");
  }
}

void SetpointProcessing::updateOpenLoop(const maelstrom_msgs::JoystickMotionCommand& msg)
{
  ROS_INFO("setpoint_processing: Sending OPEN_LOOP setpoints.");
  geometry_msgs::Wrench open_loop_msg;
  open_loop_msg.force.x  = 0.5 * MAX_FORCE_X * msg.forward;
  open_loop_msg.force.y  = 0.5 * MAX_FORCE_Y * msg.right;
  open_loop_msg.force.z  = 1.0 * MAX_FORCE_Z * msg.down;
  open_loop_msg.torque.x = 0;
  open_loop_msg.torque.y = 0.8 * MAX_TORQUE_X * msg.tilt_up;
  open_loop_msg.torque.z = 0.8 * MAX_TORQUE_Z * msg.turn_right;
  wrenchPub.publish(open_loop_msg);
}

void SetpointProcessing::updatePositionHold(const maelstrom_msgs::JoystickMotionCommand& msg)
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

void SetpointProcessing::updateDepthHold(const maelstrom_msgs::JoystickMotionCommand& msg)
{
  ROS_INFO("setpoint_processing: Sending DEPTH_HOLD setpoints.");

  ros::Time curr_time = ros::Time::now();
  double dt = (curr_time - depth_setpoint_time).toSec();
  depth_setpoint = depth_setpoint + msg.down*dt*0.1; // Magic number to control how fast setpoint moves, 0.1 should give 10 cm per sec with button fully pressed
  // Add min/max depth

  geometry_msgs::Wrench depth_hold_msg;
  depth_hold_msg.force.x  = 0.5 * MAX_FORCE_X * msg.forward;
  depth_hold_msg.force.y  = 0.5 * MAX_FORCE_Y * msg.right;
  depth_hold_msg.force.z  = 0;
  depth_hold_msg.torque.x = 0;
  depth_hold_msg.torque.y = 0.8 * MAX_TORQUE_X * msg.tilt_up;
  depth_hold_msg.torque.z = 0.8 * MAX_TORQUE_Z * msg.turn_right;
  wrenchPub.publish(depth_hold_msg);

  sensor_msgs::FluidPressure pressure_msg;
  pressure_msg.fluid_pressure = depth_setpoint * 1000 * 9.80665; // Get rid of magic numbers later, its 3 AM
  pressurePub.publish(pressure_msg);

  depth_setpoint_time = curr_time;
}

bool SetpointProcessing::healthyMessage(const maelstrom_msgs::JoystickMotionCommand& msg)
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
