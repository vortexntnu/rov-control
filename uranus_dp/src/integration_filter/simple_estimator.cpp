#include "simple_estimator.h"

#include "nav_msgs/Odometry.h"
#include <eigen_conversions/eigen_msg.h>

SimpleEstimator::SimpleEstimator()
{
  imu_sub      = nh.subscribe("imu/data", 10, &SimpleEstimator::imuCallback, this);
  pressure_sub = nh.subscribe("imu/pressure", 10, &SimpleEstimator::pressureCallback, this);
  state_pub    = nh.advertise<nav_msgs::Odometry>("state_estimate", 10);

  position.setZero();
  orientation.setIdentity();

  if (!nh.getParam("atmosphere/pressure", atmospheric_pressure))
    ROS_ERROR("Could not read parameter atmospheric pressure.");

  if (!nh.getParam("/water/density", water_density))
    ROS_ERROR("Could not read parameter water density.");

  if (!nh.getParam("/gravity/acceleration", gravitational_acceleration))
    ROS_ERROR("Could not read parameter gravititional acceleration.");
}

void SimpleEstimator::imuCallback(const sensor_msgs::Imu &msg)
{
  tf::quaternionMsgToEigen(msg.orientation, orientation);

  publish();
}

void SimpleEstimator::pressureCallback(const sensor_msgs::FluidPressure &msg)
{
  double depth = (msg.fluid_pressure - atmospheric_pressure)/(water_density * gravitational_acceleration);
  position(2) = depth;

  publish();
}

void SimpleEstimator::publish()
{
  nav_msgs::Odometry msg;
  tf::pointEigenToMsg(position, msg.pose.pose.position);
  tf::quaternionEigenToMsg(orientation, msg.pose.pose.orientation);
  state_pub.publish(msg);
}
