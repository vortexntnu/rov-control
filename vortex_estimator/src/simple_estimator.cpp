#include "vortex_estimator/simple_estimator.h"

SimpleEstimator::SimpleEstimator()
{
  imu_sub      = nh.subscribe("imu/data", 10, &SimpleEstimator::imuCallback, this);
  pressure_sub = nh.subscribe("imu/pressure", 10, &SimpleEstimator::pressureCallback, this);
  state_pub    = nh.advertise<nav_msgs::Odometry>("state_estimate", 10);

  if (!nh.getParam("atmosphere/pressure", atmospheric_pressure))
    ROS_ERROR("Could not read parameter atmospheric pressure.");

  if (!nh.getParam("/water/density", water_density))
    ROS_ERROR("Could not read parameter water density.");

  if (!nh.getParam("/gravity/acceleration", gravitational_acceleration))
    ROS_ERROR("Could not read parameter gravititional acceleration.");

  is_initialized       = false;
  imu_initialized      = false;
  pressure_initialized = false;
}

void SimpleEstimator::imuCallback(const sensor_msgs::Imu &msg)
{
  state.pose.pose.orientation = msg.orientation;
  imu_initialized = true;
  publish();
}

void SimpleEstimator::pressureCallback(const sensor_msgs::FluidPressure &msg)
{
  state.pose.pose.position.z = (msg.fluid_pressure - atmospheric_pressure)/(water_density * gravitational_acceleration);
  pressure_initialized = true;
  publish();
}

void SimpleEstimator::publish()
{
  if (is_initialized)
  {
    state_pub.publish(state);
  }
  else
  {
    if (imu_initialized && pressure_initialized)
    {
      is_initialized = true;
      ROS_INFO("Node initialized.");
      state_pub.publish(state);
    }
    else
    {
      ROS_WARN_THROTTLE(10, "Node not initialized.");
    }
  }
}
