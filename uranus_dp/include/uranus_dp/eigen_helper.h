#ifndef EIGEN_HELPER_H
#define EIGEN_HELPER_H

#include <Eigen/Dense>

template<typename Derived>
inline bool isFucked(const Eigen::MatrixBase<Derived>& x)
{
  return !((x.array() == x.array())).all() && !( (x - x).array() == (x - x).array()).all();
}

#endif
