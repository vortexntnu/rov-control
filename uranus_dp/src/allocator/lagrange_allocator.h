#ifndef LAGRANGE_ALLOCATOR_H
#define LAGRANGE_ALLOCATOR_H

#include "ros/ros.h"
#include <Eigen/Dense>
#include "geometry_msgs/Wrench.h"

#include "maelstrom_msgs/ThrusterForces.h"
#include "../eigen_typedefs.h"

class LagrangeAllocator
{
public:
  LagrangeAllocator();
  void callback(const geometry_msgs::Wrench& tauMsg);
  void setWeights(const Eigen::MatrixXd &W_new);
private:
  ros::NodeHandle nh;
  ros::Subscriber sub;
  ros::Publisher  pub;

  static const unsigned int n = 5; // Number of control forces (length of tau)
  static const unsigned int r = 6; // Number of control inputs (length of u)

  Eigen::Vector5d tau; // (n) Control forces (forces and moments on the ROV)
  Eigen::Vector6d u;   // (r) Control inputs (forces for each thruster)

  Eigen::Matrix6d    W;            // (r*r) Control force weight matrix
  Eigen::Matrix6d    K;            // (r*r) Thrust coefficient matrix
  Eigen::Matrix6d    K_inverse;    // (r*r) Inverse of K
  Eigen::Matrix5by6d T;            // (n*r) Thrust configuration matrix
  Eigen::Matrix6by5d T_geninverse; // (r*n) Generalized inverse of T

  void computeGeneralizedInverse();
};

#endif
