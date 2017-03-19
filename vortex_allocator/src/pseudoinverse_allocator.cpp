#include "pseudoinverse_allocator.h"

PseudoinverseAllocator::PseudoinverseAllocator(const Eigen::MatrixXd &T_pinv,
                                               const Eigen::MatrixXd &K_inv)
: T_pinv(T_pinv), K_inv(K_inv) {}

Eigen::VectorXd PseudoinverseAllocator::compute(const Eigen::VectorXd &tau)
{
  Eigen::VectorXd u = K_inv*T_pinv*tau;
  return u;
}
