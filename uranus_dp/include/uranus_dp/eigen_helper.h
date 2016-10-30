#ifndef EIGEN_HELPER_H
#define EIGEN_HELPER_H

#include "vortex_msgs/ThrusterForces.h"
#include "ros/ros.h"
#include <Eigen/Dense>

// Return false if X has any nan or inf elements.
template<typename Derived>
inline bool isFucked(const Eigen::MatrixBase<Derived>& X)
{
  bool has_nan = !(X.array() == X.array()).all();
  bool has_inf = !((X - X).array() == (X - X).array()).all();
  return has_nan || has_inf;
}

// Read a matrix from the ROS parameter server.
// Return false if unsuccessful.
inline bool getMatrixParam(ros::NodeHandle nh, std::string name, Eigen::MatrixXd &X)
{
  XmlRpc::XmlRpcValue matrix;
  nh.getParam(name, matrix);

  try
  {
    const int rows = matrix.size();
    const int cols = matrix[0].size();
    X.setZero(rows, cols);
    for (int i = 0; i < rows; ++i)
      for (int j = 0; j < cols; ++j)
        X(i,j) = matrix[i][j];
  }
  catch(...)
  {
    return false;
  }
  return true;
}

inline void printEigen(std::string name, const Eigen::MatrixXd &X)
{
  std::stringstream ss;
  ss << std::endl << name << " = " << std::endl << X;
  ROS_INFO_STREAM(ss.str());
}

// Calculate the pseudoinverse matrix of the matrix X.
// Return false if the calculations fails.
inline bool pseudoinverse(const Eigen::MatrixXd &X, Eigen::MatrixXd &X_pinv)
{
  Eigen::MatrixXd copy = X_pinv;
  copy = X.transpose() * (X * X.transpose()).inverse();

  if (isFucked(copy))
  {
    return false;
  }
  X_pinv = copy;
  return true;
}

// Return the 3-by-3 skew-symmetric matrix of the vector v.
inline Eigen::Matrix3d skew(const Eigen::Vector3d &v)
{
  Eigen::Matrix3d S;
  S <<  0   , -v(2),  v(1),
        v(2),  0   , -v(0),
       -v(1),  v(0),  0   ;
  return S;
}

inline void thrustEigenToMsg(const Eigen::VectorXd &u, vortex_msgs::ThrusterForces &msg)
{
  int r = u.size();
  std::vector<double> u_vec(r);
  for (int i = 0; i < r; ++i)
    u_vec[i] = u(i);
  msg.thrust = u_vec;
}

#endif
