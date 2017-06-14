#include "vortex_estimator/simple_estimator.h"

#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Dense>

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

  state.pose.pose.orientation.w = 1.0;
  state.pose.pose.orientation.x = 0.0;
  state.pose.pose.orientation.y = 0.0;
  state.pose.pose.orientation.z = 0.0;

  ROS_INFO("Initialized.");
}

void SimpleEstimator::imuCallback(const sensor_msgs::Imu &msg)
{
  // Rotation measured by IMU
  Eigen::Quaterniond quat_imu;
  tf::quaternionMsgToEigen(msg.orientation, quat_imu);
  Eigen::Vector3d euler_imu = quat_imu.toRotationMatrix().eulerAngles(2, 1, 0);

  // Alter IMU measurements to Z down, Y right, X forward
  Eigen::Vector3d euler_ned(-euler_imu(0), euler_imu(2) + PI, euler_imu(1) + PI);

  // Transform back to quaternion
  Eigen::Matrix3d R_ned;
  R_ned = Eigen::AngleAxisd(euler_ned(0), Eigen::Vector3d::UnitZ())
        * Eigen::AngleAxisd(euler_ned(1), Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(euler_ned(2), Eigen::Vector3d::UnitX());
  Eigen::Quaterniond quat_ned;

  // Convert to quaternion message and publish
  tf::quaternionEigenToMsg(quat_ned, state.pose.pose.orientation);
  state_pub.publish(state);
}

void SimpleEstimator::pressureCallback(const sensor_msgs::FluidPressure &msg)
{
  state.pose.pose.position.z = (msg.fluid_pressure - atmospheric_pressure)/(water_density * gravitational_acceleration);
  state_pub.publish(state);
}
