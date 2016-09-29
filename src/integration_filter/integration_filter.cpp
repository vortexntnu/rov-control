#include "integration_filter.h"

IntegrationFilter::IntegrationFilter()
{
  sub = nh.subscribe("imu/data", 10, &IntegrationFilter::callback, this);
  pub = nh.advertise<nav_msgs::Odometry>("state_estimate", 10);
  is_initialized = false;

  a_mn_m.setZero();
  a_mn_b.setZero();
  v_mn_b.setZero();
  v_bn_b.setZero();
  p_mn_b.setZero();
  w_bn_b.setZero();
  q_nm.setIdentity();
  q_nb.setIdentity();
  r_m_b.setZero();
  R_m_b.setIdentity();
}

bool IntegrationFilter::reset(uranus_dp::ResetIntegrationFilter::Request &req, uranus_dp::ResetIntegrationFilter::Response &resp)
{
  ROS_INFO("Resetting integration filter.");
  p_mn_b.setZero();
  v_mn_b.setZero();
  is_initialized = false;
  return true;
}

void IntegrationFilter::callback(const sensor_msgs::Imu& msg)
{
  ros::Time curr_time = msg.header.stamp;
  if (!is_initialized)
  {
    prev_time = curr_time;
    is_initialized = true;
    ROS_INFO("Initialized integration filter.");
    return;
  }

  tf::vectorMsgToEigen(msg.linear_acceleration, a_mn_m);
  tf::vectorMsgToEigen(msg.angular_velocity, w_bn_b);
  tf::quaternionMsgToEigen(msg.orientation, q_nm);
  q_nm.normalize();

  double dt = (curr_time - prev_time).toSec();
  prev_time = curr_time;
  update(dt);
  publish();
}

void IntegrationFilter::update(double dt)
{
  a_mn_b = R_m_b*a_mn_m;
  v_mn_b += dt*a_mn_b;
  p_mn_b += dt*v_mn_b;
}

void IntegrationFilter::publish()
{
  v_bn_b = v_mn_b + skew(r_m_b)*w_bn_b;
  q_nb = q_nm;

  nav_msgs::Odometry msg;
  tf::pointEigenToMsg(p_mn_b, msg.pose.pose.position);
  tf::quaternionEigenToMsg(q_nm, msg.pose.pose.orientation);
  tf::vectorEigenToMsg(v_bn_b, msg.twist.twist.linear);
  tf::vectorEigenToMsg(w_bn_b, msg.twist.twist.angular);
  pub.publish(msg);
}

Eigen::Matrix3d IntegrationFilter::skew(const Eigen::Vector3d& v)
{
  Eigen::Matrix3d S;
  S <<  0   , -v(2),  v(1),
        v(2),  0   , -v(0),
       -v(1),  v(0),  0   ;
  return S;
}
