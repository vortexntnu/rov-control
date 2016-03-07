#ifndef LAGRANGE_ALLOCATOR_H
#define LAGRANGE_ALLOCATOR_H

#include "ros/ros.h"
#include <Eigen/Dense>
#include "geometry_msgs/Wrench.h"
#include "uranus_dp/ThrusterForces.h"

class LagrangeAllocator
{
public:
    LagrangeAllocator();
    void tauCallback(const geometry_msgs::Wrench& tauMsg);
    void setWeights(const Eigen::MatrixXd &W_new);
private:
    ros::NodeHandle nh;
    ros::Subscriber tauSub;
    ros::Publisher  uPub;

    // Move to param server
    unsigned int n; // Number of control forces (length of tau)
    unsigned int r; // Number of control inputs (length of u)

    Eigen::MatrixXd W;            // (r*r) Control force weight matrix
    Eigen::MatrixXd K;            // (r*r) Thrust coefficient matrix
    Eigen::MatrixXd K_inverse;    // (r*r) Inverse of K
    Eigen::MatrixXd T;            // (n*r) Thrust configuration matrix
    Eigen::MatrixXd T_geninverse; // (r*n) Generalized inverse of T

    void computeGeneralizedInverse();
};

#endif
