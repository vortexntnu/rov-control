#ifndef SETPOINTS_H
#define SETPOINTS_H

#include <Eigen/Dense>
#include "uranus_dp/eigen_typedefs.h"
#include "uranus_dp/control_mode_enum.h"

class Setpoints
{
public:
  Setpoints(const Eigen::Vector6d &wrench_scaling,
            const Eigen::Vector6d &wrench_max,
            const Eigen::Vector6d &pose_rate);
  bool update(const double time, const Eigen::Vector6d &command);
  void get(Eigen::Vector6d &wrench);
  void get(Eigen::Vector3d    &position,
           Eigen::Quaterniond &orientation);
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
};

#endif
