// Implement the unweighted pseudoinverse-based allocator described in e.g.
// Fossen 2011 Handbook of Marine Craft Hydrodynamics and Motion Control
// (chapter 12.3.2).

#ifndef PSEUDOINVERSE_ALLOCATOR_H
#define PSEUDOINVERSE_ALLOCATOR_H

#include "uranus_dp/eigen_helper.h"
#include "uranus_dp/eigen_typedefs.h"
#include "vortex_msgs/ThrusterForces.h"

#include "ros/ros.h"
#include "geometry_msgs/Wrench.h"

#include <Eigen/Dense>

class PseudoinverseAllocator
{
public:
  PseudoinverseAllocator();
  void callback(const geometry_msgs::Wrench& tau_msg);
private:
  ros::NodeHandle nh;
  ros::Subscriber sub;
  ros::Publisher  pub;

  double max_thrust;
  double min_thrust;

  int n; // Number of control forces (length of tau)
  int r; // Number of control inputs (length of u)
  std::map<std::string, bool> dofs; // Map of controllable dofs

  Eigen::VectorXd tau;    // (n) Control forces (forces and moments on the ROV)
  Eigen::VectorXd u;      // (r) Control inputs (forces for each thruster)
  Eigen::MatrixXd T;      // (n*r) Thrust configuration matrix
  Eigen::MatrixXd T_pinv; // (r*n) Generalized inverse of T
  Eigen::MatrixXd K_inv;  // (r*r) Inverse thrust coefficient matrix

  void getParams();
};

#endif
