#include "vortex_controller/setpoints.h"

Setpoints::Setpoints(const Eigen::Vector6d &wrench_scaling,
                     const Eigen::Vector6d &wrench_max,
                     const Eigen::Vector6d &pose_rate)
: wrench_scaling_(wrench_scaling),
  wrench_max_(wrench_max),
  pose_rate_(pose_rate)
{
  wrench_.setZero();
  position_.setZero();
  orientation_.setIdentity();

  time_valid_   = false;
  wrench_valid_ = false;
  pose_valid_   = false;
}

bool Setpoints::update(const double time, const Eigen::Vector6d &command)
{
  // Update wrench setpoint (independent of timestamp)
  for (int i = 0; i < 6; ++i)
    wrench_(i) = wrench_scaling_(i) * wrench_max_(i) * command(i);
  wrench_valid_ = true;

  // Check difference and validity of timestamp
  if (!time_valid_)
  {
    time_ = time;
    time_valid_ = true;
    return false;
  }
  double dt = time - time_;
  time_ = time;
  if (dt == 0)
    return false;

  // Increment position setpoint
  for (int i = 0; i < 3; ++i)
    position_(i) += pose_rate_(i) * dt * command(i);

  // Convert quaternion setpoint to euler angles (ZYX convention)
  Eigen::Vector3d euler;
  euler = orientation_.toRotationMatrix().eulerAngles(2, 1, 0);

  // Increment euler setpoint
  for (int i = 0; i < 3; ++i)
    euler(i) += pose_rate_(i) * dt * command(3+i);

  // Convert euler setpoint back to quaternions
  Eigen::Matrix3d R;
  R = Eigen::AngleAxisd(euler(0), Eigen::Vector3d::UnitZ())
    * Eigen::AngleAxisd(euler(1), Eigen::Vector3d::UnitY())
    * Eigen::AngleAxisd(euler(2), Eigen::Vector3d::UnitX());
  Eigen::Quaterniond q(R);
  orientation_ = q;

  pose_valid_ = true;
  return true;
}

bool Setpoints::get(Eigen::Vector6d *wrench)
{
  if (!wrench_valid_)
    return false;

  *wrench = wrench_;
  return true;
}

bool Setpoints::get(Eigen::Vector3d    *position,
                    Eigen::Quaterniond *orientation)
{
  if (!pose_valid_)
    return false;

  *position    = position_;
  *orientation = orientation_;
  return true;
}

void Setpoints::set(const Eigen::Vector3d    &position,
                    const Eigen::Quaterniond &orientation)
{
  position_    = position;
  orientation_ = orientation;

  time_valid_ = false;
  pose_valid_ = true;
}
