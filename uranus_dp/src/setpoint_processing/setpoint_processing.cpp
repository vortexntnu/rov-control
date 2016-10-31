#include "setpoint_processing.h"

#include "uranus_dp/ResetEstimator.h"
#include "uranus_dp/SetControlMode.h"

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Wrench.h"

#include <tf/transform_datatypes.h>
#include <eigen_conversions/eigen_msg.h>

SetpointProcessing::SetpointProcessing()
{
  command_sub  = nh.subscribe("joystick_motion_command", 10, &SetpointProcessing::commandCallback, this);
  state_sub    = nh.subscribe("state_estimate", 10, &SetpointProcessing::stateCallback, this);
  wrench_pub   = nh.advertise<geometry_msgs::Wrench>("wrench_setpoints", 10);
  pose_pub     = nh.advertise<geometry_msgs::Pose>("pose_setpoints", 10);
  mode_client  = nh.serviceClient<uranus_dp::SetControlMode>("set_control_mode");
  reset_client = nh.serviceClient<uranus_dp::ResetEstimator>("reset_estimator");

  control_mode = ControlModes::OPEN_LOOP;
  prev_time_valid = false;

  pose.setZero();
  pose_setpoint.setZero();
  wrench_setpoint.setZero();

  getParams();
}

void SetpointProcessing::commandCallback(const vortex_msgs::JoystickMotionCommand& msg)
{
  if (!healthyMessage(msg))
  {
    ROS_WARN("setpoint_processing: Joystick motion command message out of range, ignoring...");
    return;
  }

  if (msg.control_mode != control_mode)
  {
    control_mode = static_cast<ControlMode>(msg.control_mode);
    switch (control_mode)
    {
      case ControlModes::OPEN_LOOP:
      break;

      case ControlModes::POSITION_HOLD:
      pose_setpoint = pose;
      prev_time_valid = false;
      break;
    }

    uranus_dp::SetControlMode srv;
    srv.request.mode = control_mode;
    if (!mode_client.call(srv))
      ROS_ERROR_STREAM("Failed to call service set_control_mode. New mode " << control_mode << " not activated.");
  }

  switch (control_mode)
  {
    case ControlModes::OPEN_LOOP:
    updateWrenchSetpoints(msg);
    publishWrenchSetpoints();
    break;

    case ControlModes::POSITION_HOLD:
    updatePoseSetpoints(msg);
    publishPoseSetpoints();
    break;
  }
}

void SetpointProcessing::stateCallback(const nav_msgs::Odometry &msg)
{
  pose(0) = msg.pose.pose.position.x;
  pose(1) = msg.pose.pose.position.y;
  pose(2) = msg.pose.pose.position.z;

  tf::Quaternion q;
  tf::quaternionMsgToTF(msg.pose.pose.orientation, q); q.normalize();
  tf::Matrix3x3 m(q);
  m.getRPY(pose(3), pose(4), pose(5));
}

void SetpointProcessing::updatePoseSetpoints(const vortex_msgs::JoystickMotionCommand& msg)
{
  if (!prev_time_valid)
  {
    prev_time = msg.header.stamp;
    prev_time_valid = true;
    return;
  }

  // Calculate time difference
  ros::Time curr_time = msg.header.stamp;
  double dt = (curr_time - prev_time).toSec();
  prev_time = curr_time;

  if (dt == 0)
    ROS_WARN("Zero time difference between propulsion command messages.");

  // Increment setpoints (position and euler angle orientation)
  pose_setpoint(0) += pose_command_rate[0] * dt * msg.forward;
  pose_setpoint(1) += pose_command_rate[1] * dt * msg.right;
  pose_setpoint(2) += pose_command_rate[2] * dt * msg.down;
  pose_setpoint(3) += pose_command_rate[3] * dt * msg.roll_right;
  pose_setpoint(4) += pose_command_rate[4] * dt * msg.tilt_up;
  pose_setpoint(5) += pose_command_rate[5] * dt * msg.turn_right;
}

void SetpointProcessing::publishPoseSetpoints()
{
  // Convert euler angle setpoints to quaternions
  tf::Quaternion q_setpoint;
  q_setpoint.setRPY(pose_setpoint(3), pose_setpoint(4), pose_setpoint(5));
  // Create and publish pose setpoint message
  geometry_msgs::Pose pose_msg;
  tf::pointEigenToMsg(pose_setpoint.head(3), pose_msg.position);
  tf::quaternionTFToMsg(q_setpoint, pose_msg.orientation);
  pose_pub.publish(pose_msg);
}

void SetpointProcessing::updateWrenchSetpoints(const vortex_msgs::JoystickMotionCommand& msg)
{
  wrench_setpoint(0) = wrench_command_scaling[0] * wrench_command_max[0] * msg.forward;
  wrench_setpoint(1) = wrench_command_scaling[1] * wrench_command_max[1] * msg.right;
  wrench_setpoint(2) = wrench_command_scaling[2] * wrench_command_max[2] * msg.down;
  wrench_setpoint(3) = wrench_command_scaling[3] * wrench_command_max[3] * msg.roll_right;
  wrench_setpoint(4) = wrench_command_scaling[4] * wrench_command_max[4] * msg.tilt_up;
  wrench_setpoint(5) = wrench_command_scaling[5] * wrench_command_max[5] * msg.turn_right;
}

void SetpointProcessing::publishWrenchSetpoints()
{
  geometry_msgs::Wrench wrench_msg;
  tf::wrenchEigenToMsg(wrench_setpoint, wrench_msg);
  wrench_pub.publish(wrench_msg);
}

void SetpointProcessing::getParams()
{
  // Read control command parameters
  if (!nh.getParam("/control/command/wrench/max", wrench_command_max))
    ROS_FATAL("Failed to read parameter max wrench command.");
  if (!nh.getParam("/control/command/wrench/scaling", wrench_command_scaling))
    ROS_FATAL("Failed to read parameter scaling wrench command.");
  if (!nh.getParam("/control/command/pose/rate", pose_command_rate))
    ROS_FATAL("Failed to read parameter pose command rate.");
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
