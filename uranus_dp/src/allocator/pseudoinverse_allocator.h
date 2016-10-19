#ifndef PSEUDOINVERSE_ALLOCATOR_H
#define PSEUDOINVERSE_ALLOCATOR_H

#include "ros/ros.h"
#include <Eigen/Dense>
#include "geometry_msgs/Wrench.h"

#include "vortex_msgs/ThrusterForces.h"
#include "uranus_dp/eigen_typedefs.h"

class PseudoinverseAllocator
{
public:
  PseudoinverseAllocator();
  void callback(const geometry_msgs::Wrench& tauMsg);
  void setWeights(const Eigen::MatrixXd &W_new);
private:
  ros::NodeHandle nh;
  ros::Subscriber sub;
  ros::Publisher  pub;

  int n; // Number of control forces (length of tau)
  int r; // Number of control inputs (length of f)

  Eigen::Vector5d tau; // (n) Control forces (forces and moments on the ROV)
  Eigen::Vector6d f;   // (r) Control inputs (forces for each thruster)

  Eigen::Matrix6d    K_inv;    // (r*r) Inverse thrust coefficient matrix
  Eigen::MatrixXd    T;            // (n*r) Thrust configuration matrix
  Eigen::Matrix6by5d T_pinv; // (r*n) Generalized inverse of T

  void computePseudoinverse();
};

#endif
