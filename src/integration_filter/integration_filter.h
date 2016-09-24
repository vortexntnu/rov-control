#ifndef INTEGRATION_FILTER_H
#define INTEGRATION_FILTER_H

#include "ros/ros.h"
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "uranus_dp/ResetIntegrationFilter.h"
#include "../eigen_typedefs.h"

class IntegrationFilter
{
public:
  IntegrationFilter();
  bool reset(uranus_dp::ResetIntegrationFilter::Request &req, uranus_dp::ResetIntegrationFilter::Response &resp);
  void callback(const sensor_msgs::Imu& msg);
  void update(double dt);
  void publish();
private:
  ros::NodeHandle nh;
  ros::Subscriber sub;
  ros::Publisher  pub;

  bool is_initialized;
  ros::Time prev_time;

  Eigen::Vector3d    a_mn_m; // [m/s^2] Acceleration of {m} wrt. {n} in {m}
  Eigen::Vector3d    a_mn_b; // [m/s^2] Acceleration of {m} wrt. {n} in {b}
  Eigen::Vector3d    v_mn_b; // [m/s] Velocity estimate of {m} wrt. {n} in {b}
  Eigen::Vector3d    v_bn_b; // [m/s] Velocity estimate of {b} wrt. {n} in {b}
  Eigen::Vector3d    p_mn_b; // [m] Position estimate of {m} wrt. {n} in {b}
  Eigen::Vector3d    w_bn_b; // [rad/s] Ang. velocity of {b} wrt. {n} in {b}
  Eigen::Quaterniond q_nm;   // [1] Quaternion orientation from {n} to {m}
  Eigen::Quaterniond q_nb;   // [1] Quaternion orientation from {n} to {b}
  Eigen::Vector3d    r_m_b;  // [m] Position of {m} relative to {b}
  Eigen::Matrix3d    R_m_b;  // [?] Rotation matrix from {m} to {b}

  Eigen::Matrix3d skew(const Eigen::Vector3d& v);
};

#endif
