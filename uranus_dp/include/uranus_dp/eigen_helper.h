#ifndef EIGEN_HELPER_H
#define EIGEN_HELPER_H

#include "ros/ros.h"
#include <Eigen/Dense>

template<typename Derived>
inline bool isFucked(const Eigen::MatrixBase<Derived>& x)
{
  return !((x.array() == x.array())).all() && !( (x - x).array() == (x - x).array()).all();
}

inline Eigen::MatrixXd getMatrixParam(ros::NodeHandle nh, std::string name)
{
  XmlRpc::XmlRpcValue matrix;
  nh.getParam(name, matrix);

  try
  {
    const int rows = matrix.size();
    const int cols = matrix[0].size();
    Eigen::MatrixXd X(rows,cols);
    for (int i = 0; i < rows; ++i)
      for (int j = 0; j < cols; ++j)
        X(i,j) = matrix[i][j];
    return X;
  }
  catch(...)
  {
    ROS_ERROR("Error in getMatrixParam. Returning 1-by-1 zero matrix.");
    return Eigen::MatrixXd::Zero(1,1);
  }
}

inline void printEigen(std::string name, const Eigen::MatrixXd &x)
{
  std::stringstream ss;
  ss << name << " = " << std::endl << x;
  ROS_INFO_STREAM(ss.str());
}

inline Eigen::MatrixXd pinv(const Eigen::MatrixXd &X)
{
  Eigen::MatrixXd X_pinv = X.transpose() * ( X*X.transpose() ).inverse();

  if (isFucked(X_pinv))
  {
    ROS_WARN("Could not compute pseudoinverse. Returning transpose.");
    return X.transpose();
  }

  return X_pinv;
}

#endif
