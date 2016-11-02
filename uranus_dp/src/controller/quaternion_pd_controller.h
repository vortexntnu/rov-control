// An implementation of
// Fjellstad & Fossen 1994: Quaternion Feedback Regulation of Underwater Vehicles,
// a nonlinear PD position and orientation controller.
// All variables are named as they appear in the paper.

#ifndef QUATERNION_PD_CONTROLLER_H
#define QUATERNION_PD_CONTROLLER_H

#include <Eigen/Dense>
#include "uranus_dp/eigen_typedefs.h"

class QuaternionPdController
{
public:
  QuaternionPdController(double a,
                         double b,
                         double c,
                         double W,
                         double B,
                         const Eigen::Vector3d r_G,
                         const Eigen::Vector3d r_B);
  void setGains(double a, double b, double c);
  Eigen::Vector6d compute(Eigen::Vector3d x, Eigen::Quaterniond q, Eigen::Vector6d nu, Eigen::Vector3d x_d, Eigen::Quaterniond q_d);
private:
  Eigen::Matrix6d proportionalGainMatrix(Eigen::Matrix3d R);
  Eigen::Vector6d errorVector(Eigen::Vector3d p, Eigen::Vector3d p_d, Eigen::Quaterniond q, Eigen::Quaterniond q_d);
  Eigen::Vector6d restoringForceVector(Eigen::Matrix3d R);
  int             sgn(double x);

  double c;            // Orientation gain
  Eigen::Matrix6d K_d; // Derivative gain matrix
  Eigen::Matrix3d K_x; // Position error gain matrix

  Eigen::Vector3d r_G; // Center of gravity, expressed in body frame
  Eigen::Vector3d r_B; // Center of buoyancy, expressed in body frame
  double W;            // [N] Weight of ROV
  double B;            // [N] Buoyancy of ROV
};

#endif
