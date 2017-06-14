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

  ROS_INFO("Node initialized.");
}

void SimpleEstimator::imuCallback(const sensor_msgs::Imu &msg)
{
  // IMU orientation is in north-west-up
  // Must rotate to give orientation in north-east-down

  // Rotation from ROV north-west-up frame to IMU
  Eigen::Quaterniond q_nwu_imu;
  tf::quaternionMsgToEigen(msg.orientation, q_nwu_imu);
  Eigen::Matrix3d R_nwu_imu = q_nwu_imu.toRotationMatrix();

  // Rotation from north-west-up to north-east-down
  Eigen::Matrix3d R_ned_nwu;
  R_ned_nwu << 1.0,  0.0,  0.0,
               0.0, -1.0,  0.0,
               0.0,  0.0, -1.0;

  // Rotation from north-east-down to imu
  Eigen::Matrix3d R_ned_imu = R_ned_nwu * R_ned_imu;
  Eigen::Quaterniond q_ned_imu(R_ned_imu);

  tf::quaternionEigenToMsg(q_ned_imu, state.pose.pose.orientation);
  state_pub.publish(state);
}

void SimpleEstimator::pressureCallback(const sensor_msgs::FluidPressure &msg)
{
  state.pose.pose.position.z = (msg.fluid_pressure - atmospheric_pressure)/(water_density * gravitational_acceleration);
  state_pub.publish(state);
}
