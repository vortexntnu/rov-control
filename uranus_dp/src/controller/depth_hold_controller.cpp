#include "depth_hold_controller.h"

DepthHoldController::DepthHoldController()
{
  pressureSub = nh.subscribe("imu/pressure", 10, &DepthHoldController::pressureCallback, this);
  setpointSub = nh.subscribe("pressure_setpoint", 10, &DepthHoldController::setpointCallback, this);
  openloopSub = nh.subscribe("wrench_setpoints", 10, &DepthHoldController::openloopCallback, this);
  controlPub  = nh.advertise<geometry_msgs::Wrench>("rov_forces", 10);
  enabled = false;

  k_p = 1e-3; // 1e-3 gives roughly 10 Newton force in heave at 1 meter depth error
}

void DepthHoldController::pressureCallback(const sensor_msgs::FluidPressure& msg)
{
  pressure_reading = msg.fluid_pressure;
}

void DepthHoldController::setpointCallback(const sensor_msgs::FluidPressure& msg)
{
  pressure_setpoint = msg.fluid_pressure;
}

void DepthHoldController::openloopCallback(const geometry_msgs::Wrench& msg)
{
  surge_command = msg.force.x;
  sway_command  = msg.force.y;
  pitch_command = msg.torque.y;
  yaw_command   = msg.torque.z;
}

void DepthHoldController::compute()
{
  if (enabled)
  {
    double heave_command = (pressure_setpoint - pressure_reading) * k_p; // Positive heave command when we must go deeper

    geometry_msgs::Wrench msg;
    msg.force.x = surge_command;
    msg.force.y = sway_command;
    msg.force.z = heave_command;
    msg.torque.x = 0;
    msg.torque.y = pitch_command;
    msg.torque.z = yaw_command;
    controlPub.publish(msg);
  }
}

void DepthHoldController::enable()
{
  enabled = true;
}

void DepthHoldController::disable()
{
  enabled = false;
}
