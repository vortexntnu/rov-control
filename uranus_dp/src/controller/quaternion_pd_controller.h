// Based on Fjellstad & Fossen 1994: Quaternion Feedback Regulation of Underwater Vehicles

#ifndef QUATERNION_PD_CONTROLLER_H
#define QUATERNION_PD_CONTROLLER_H

#include "ros/ros.h"
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/Pose.h"

#include "../eigen_typedefs.h"

class QuaternionPdController
{
public:
  QuaternionPdController();
  void stateCallback(const nav_msgs::Odometry &msg);
  void setpointCallback(const geometry_msgs::Pose &msg);
  void setGains(double a, double b, double c);
  void compute();
  void enable();
  void disable();
private:
  ros::NodeHandle nh;
  ros::Subscriber stateSub;
  ros::Subscriber setpointSub;
  ros::Publisher  controlPub;
  bool enabled;

  void updateProportionalGainMatrix();
  void updateErrorVector();
  void updateRestoringForceVector();

  int             sgn(double x);
  Eigen::Matrix3d skew(const Eigen::Vector3d &v);

  Eigen::Vector3d    p;   // Position state
  Eigen::Quaterniond q;   // Orientation state
  Eigen::Vector6d    nu;  // Velocity state (linear and angular)
  Eigen::Vector3d    p_d; // Desired position
  Eigen::Quaterniond q_d; // Desired attitude
  Eigen::Vector6d    z;   // Pose error

  double a;            // Derivative gain
  double b;            // Position gain
  double c;            // Orientation gain
  Eigen::Matrix6d K_P; // Proportional gain matrix
  Eigen::Matrix6d K_D; // Derivative gain matrix
  Eigen::Matrix3d K_p; // Position error gain matrix (submatrix of K_P)

  Eigen::Vector6d tau; // Control ROV forces

  Eigen::Vector6d g; // Restoring force vector
  Eigen::Matrix3d R; // Rotation matrix from {n} to {b}

  Eigen::Vector3d r_g; // Center of gravity, expressed in body frame
  Eigen::Vector3d r_b; // Center of buoyancy, expressed in body frame
  double W;            // [N] Weight of ROV
  double B;            // [N] Buoyancy of ROV
};

#endif
