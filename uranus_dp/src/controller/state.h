#ifndef STATE_H
#define STATE_H

#include <Eigen/Dense>
#include "uranus_dp/eigen_typedefs.h"

class State
{
public:
  State();
  bool get(Eigen::Vector3d    &position,
           Eigen::Quaterniond &orientation);
  bool get(Eigen::Vector3d    &position,
           Eigen::Quaterniond &orientation,
           Eigen::Vector6d    &velocity);
  void set(const Eigen::Vector3d    &position,
           const Eigen::Quaterniond &orientation,
           const Eigen::Vector6d    &velocity);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
  Eigen::Vector3d    position_;
  Eigen::Quaterniond orientation_;
  Eigen::Vector6d    velocity_;

  bool is_initialized_;
};

#endif
