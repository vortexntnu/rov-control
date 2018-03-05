#include "vortex_controller/state.h"

State::State()
{
  m_position.setZero();
  m_orientation.setIdentity();
  m_velocity.setZero();
  m_is_initialized = false;
}

bool State::get(Eigen::Vector3d    *position,
                Eigen::Quaterniond *orientation)
{
  if (!m_is_initialized)
    return false;

  *position    = m_position;
  *orientation = m_orientation;
  return true;
}

bool State::get(Eigen::Vector3d    *position,
                Eigen::Quaterniond *orientation,
                Eigen::Vector6d    *velocity)
{
  if (!m_is_initialized)
    return false;

  *position    = m_position;
  *orientation = m_orientation;
  *velocity    = m_velocity;
  return true;
}

bool State::get(Eigen::Vector3d *position)
{
  if (!m_is_initialized)
    return false;

  *position = m_position;
}

bool State::get(Eigen::Quaterniond *orientation)
{
  if (!m_is_initialized)
    return false;

  *orientation = m_orientation;
}

void State::set(const Eigen::Vector3d    &position,
                const Eigen::Quaterniond &orientation,
                const Eigen::Vector6d    &velocity)
{
  m_position    = position;
  m_orientation = orientation;
  m_velocity    = velocity;

  m_is_initialized = true;
}
