#include "vortex_estimator/simple_estimator.h"

SimpleEstimator::SimpleEstimator()
{
  imu_sub      = nh.subscribe("/sensors/imu/data", 10, &SimpleEstimator::imuCallback, this);
  pressure_sub = nh.subscribe("/sensors/pressure", 10, &SimpleEstimator::pressureCallback, this);
  state_pub    = nh.advertise<nav_msgs::Odometry>("state_estimate", 10);

  if (!nh.getParam("atmosphere/pressure", atmospheric_pressure))
    ROS_ERROR("Could not read parameter atmospheric pressure.");

  if (!nh.getParam("/water/density", water_density))
    ROS_ERROR("Could not read parameter water density.");

  if (!nh.getParam("/gravity/acceleration", gravitational_acceleration))
    ROS_ERROR("Could not read parameter gravititional acceleration.");
}

void SimpleEstimator::imuCallback(const sensor_msgs::Imu &msg)
{
  state.pose.pose.orientation = msg.orientation;
  state_pub.publish(state);
}

void SimpleEstimator::pressureCallback(const sensor_msgs::FluidPressure &msg)
{
  state.pose.pose.position.z = (msg.fluid_pressure - atmospheric_pressure)/(water_density * gravitational_acceleration);
  state_pub.publish(state);
}
