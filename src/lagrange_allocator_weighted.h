#ifndef LAGRANGE_ALLOCATOR_WEIGHTED_H
#define LAGRANGE_ALLOCATOR_WEIGHTED_H

#include "ros/ros.h"
#include <Eigen/Dense>
#include "geometry_msgs/Wrench.h"
#include "uranus_dp/ThrusterForces.h"

class LagrangeAllocatorWeighted
{
public:
    LagrangeAllocatorWeighted();
    void tauCallback(const geometry_msgs::Wrench& tauMsg);
private:
    ros::NodeHandle nh;
    ros::Subscriber tauSub;
    ros::Publisher  uPub;

    // Move to param server
    unsigned int n; // Number of control forces (length of tau)
    unsigned int r; // Number of control inputs (length of u)

    Eigen::VectorXd tau;          // Desired control forces
    Eigen::VectorXd u;            // Desired control inputs
    Eigen::MatrixXd W;            // Control force weight matrix
    Eigen::MatrixXd K;            // Thrust coefficient matrix
    Eigen::MatrixXd K_inverse;    // Inverse of K
    Eigen::MatrixXd T;            // Thrust configuration matrix
    Eigen::MatrixXd T_geninverse; // Generalized inverse of T
};

#endif
