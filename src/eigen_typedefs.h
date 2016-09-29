#ifndef EIGEN_TYPEDEFS_H
#define EIGEN_TYPEDEFS_H

#include <Eigen/Dense>

namespace Eigen
{
typedef Eigen::Matrix<double,6,6> Matrix6d;
typedef Eigen::Matrix<double,6,5> Matrix6by5d;
typedef Eigen::Matrix<double,5,6> Matrix5by6d;

typedef Eigen::Matrix<double,6,1> Vector6d;
typedef Eigen::Matrix<double,5,1> Vector5d;
typedef Eigen::Matrix<double,4,1> Vector4d;
}

#endif
