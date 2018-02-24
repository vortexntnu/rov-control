#include "vortex_controller/setpoints.h"

Setpoints::Setpoints(const Eigen::Vector6d &wrench_scaling,
                     const Eigen::Vector6d &wrench_max,
                     const Eigen::Vector6d &pose_rate)
: m_wrench_scaling(wrench_scaling),
  m_wrench_max(wrench_max),
  m_pose_rate(pose_rate)
{
  m_wrench.setZero();
  m_position.setZero();
  m_orientation.setIdentity();

  m_time_is_valid   = false;
  m_wrench_is_valid = false;
  m_pose_is_valid   = false;
}

bool Setpoints::update(const double time, const Eigen::Vector6d &command)
{
  // Update wrench setpoint (independent of timestamp)
  for (int i = 0; i < 6; ++i)
    m_wrench(i) = m_wrench_scaling(i) * m_wrench_max(i) * command(i);
  m_wrench_is_valid = true;

#if 0
  // Check difference and validity of timestamp
  if (!m_time_is_valid)
  {
    m_time = time;
    m_time_is_valid = true;
    return false;
  }
  double dt = time - m_time;
  m_time = time;
  if (dt == 0)
    return false;

  // Increment position setpoint
  for (int i = 0; i < 3; ++i)
    m_position(i) += m_pose_rate(i) * dt * command(i);

  // Convert quaternion setpoint to euler angles (ZYX convention)
  Eigen::Vector3d euler;
  euler = m_orientation.toRotationMatrix().eulerAngles(2, 1, 0);

  // Increment euler setpoint
  for (int i = 0; i < 3; ++i)
    euler(i) += m_pose_rate(i) * dt * command(3+i);

  // Convert euler setpoint back to quaternions
  Eigen::Matrix3d R;
  R = Eigen::AngleAxisd(euler(0), Eigen::Vector3d::UnitZ())
    * Eigen::AngleAxisd(euler(1), Eigen::Vector3d::UnitY())
    * Eigen::AngleAxisd(euler(2), Eigen::Vector3d::UnitX());
  Eigen::Quaterniond q(R);
  m_orientation = q;

  m_pose_is_valid = true;
#endif
  return true;
}

bool Setpoints::get(Eigen::Vector6d *wrench)
{
  if (!m_wrench_is_valid)
    return false;

  *wrench = m_wrench;
  return true;
}

bool Setpoints::get(Eigen::Vector3d    *position,
                    Eigen::Quaterniond *orientation)
{
  if (!m_pose_is_valid)
    return false;

  *position    = m_position;
  *orientation = m_orientation;
  return true;
}

void Setpoints::set(const Eigen::Vector3d    &position,
                    const Eigen::Quaterniond &orientation)
{
  m_position    = position;
  m_orientation = orientation;

  m_time_is_valid = false;
  m_pose_is_valid = true;
}
