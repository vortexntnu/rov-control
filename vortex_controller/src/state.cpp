#include "vortex_controller/state.h"

State::State()
{
  position_.setZero();
  orientation_.setIdentity();
  velocity_.setZero();
  is_initialized_ = false;
}

bool State::get(Eigen::Vector3d    *position,
                Eigen::Quaterniond *orientation)
{
  if (!is_initialized_)
    return false;

  *position    = position_;
  *orientation = orientation_;
  return true;
}

bool State::get(Eigen::Vector3d    *position,
                Eigen::Quaterniond *orientation,
                Eigen::Vector6d    *velocity)
{
  if (!is_initialized_)
    return false;

  *position    = position_;
  *orientation = orientation_;
  *velocity    = velocity_;
  return true;
}

void State::set(const Eigen::Vector3d    &position,
                const Eigen::Quaterniond &orientation,
                const Eigen::Vector6d    &velocity)
{
  position_    = position;
  orientation_ = orientation;
  velocity_    = velocity;

  is_initialized_ = true;
}
