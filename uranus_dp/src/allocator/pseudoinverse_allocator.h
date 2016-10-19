#ifndef PSEUDOINVERSE_ALLOCATOR_H
#define PSEUDOINVERSE_ALLOCATOR_H

#include "ros/ros.h"
#include <Eigen/Dense>
#include "geometry_msgs/Wrench.h"

#include "vortex_msgs/ThrusterForces.h"
#include "uranus_dp/eigen_typedefs.h"
#include "uranus_dp/eigen_helper.h"

class PseudoinverseAllocator
{
public:
  PseudoinverseAllocator();
  void callback(const geometry_msgs::Wrench& tauMsg);
private:
  ros::NodeHandle nh;
  ros::Subscriber sub;
  ros::Publisher  pub;

  int n; // Number of control forces (length of tau)
  int r; // Number of control inputs (length of f)

  Eigen::VectorXd tau;    // (n) Control forces (forces and moments on the ROV)
  Eigen::VectorXd u;      // (r) Control inputs (forces for each thruster)
  Eigen::MatrixXd T;      // (n*r) Thrust configuration matrix
  Eigen::MatrixXd T_pinv; // (r*n) Generalized inverse of T
  Eigen::MatrixXd K_inv;  // (r*r) Inverse thrust coefficient matrix
};

#endif
