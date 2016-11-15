#include "state.h"

State::State()
{
  position_.setZero();
  orientation_.setIdentity();
  velocity_.setZero();

  is_initialized = false;
}

void State::set(const Eigen::Vector3d    &position,
                const Eigen::Quaterniond &orientation,
                const Eigen::Vector6d    &velocity)
{
  position_    = position;
  orientation_ = orientation;
  velocity_    = velocity;

  if (!is_initialized)
    is_initialized = true;
}

bool State::get(Eigen::Vector3d    &position,
                Eigen::Quaterniond &orientation,
                Eigen::Vector6d    &velocity)
{
  if (!is_initialized)
    return false;

  position    = position_;
  orientation = orientation_;
  velocity    = velocity_;

  return true;
}
