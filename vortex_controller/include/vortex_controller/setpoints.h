#ifndef VORTEX_CONTROLLER_SETPOINTS_H
#define VORTEX_CONTROLLER_SETPOINTS_H

#include <Eigen/Dense>
#include "vortex/eigen_typedefs.h"

class Setpoints
{
public:
  Setpoints(const Eigen::Vector6d &wrench_scaling,
            const Eigen::Vector6d &wrench_max,
            const Eigen::Vector6d &pose_rate);
  bool update(const double time, const Eigen::Vector6d &command);
  bool get(Eigen::Vector6d *wrench);
  bool get(Eigen::Vector3d    *position,
           Eigen::Quaterniond *orientation);
  void set(const Eigen::Vector3d    &position,
           const Eigen::Quaterniond &orientation);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
  Eigen::Vector6d    wrench_;
  Eigen::Vector3d    position_;
  Eigen::Quaterniond orientation_;

  Eigen::Vector6d wrench_scaling_;
  Eigen::Vector6d wrench_max_;
  Eigen::Vector6d pose_rate_;

  double time_;
  bool   time_valid_;
  bool   wrench_valid_;
  bool   pose_valid_;
};

#endif  // VORTEX_CONTROLLER_SETPOINTS_H
