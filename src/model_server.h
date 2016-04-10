#ifndef MODEL_SERVER_H
#define MODEL_SERVER_H

#include "ros/ros.h"
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Dense>
#include "eigen_typedefs.h"

Eigen::Matrix6d getM();
Eigen::Matrix6d getC();
Eigen::Matrix6d getD();
Eigen::Vector6d getG();
Eigen::Matrix3d getR(Eigen::Quaterniond q);
Eigen::Matrix<double,4,3> getT(Eigen::Quaterniond q);
Eigen::Matrix<double,7,6> getJ(Eigen::Quaterniond q);

#endif
