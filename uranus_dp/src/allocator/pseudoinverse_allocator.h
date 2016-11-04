// Implement the unweighted pseudoinverse-based allocator described in e.g.
// Fossen 2011 Handbook of Marine Craft Hydrodynamics and Motion Control
// (chapter 12.3.2).

#ifndef PSEUDOINVERSE_ALLOCATOR_H
#define PSEUDOINVERSE_ALLOCATOR_H

#include <Eigen/Dense>

class PseudoinverseAllocator
{
public:
  PseudoinverseAllocator(const Eigen::MatrixXd &T_pinv,
                         const Eigen::MatrixXd &K_inv);
  Eigen::VectorXd compute(const Eigen::VectorXd &tau);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
  Eigen::MatrixXd T_pinv; // Pseudoinverse of thrust configuration matrix
  Eigen::MatrixXd K_inv;  // TODO: what does this matrix even mean
};

#endif
